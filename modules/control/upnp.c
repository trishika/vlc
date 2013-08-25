/*****************************************************************************
 * upnp.cpp : upnp control module for vlc
 *****************************************************************************
 * Copyright (C) 2013 the VideoLAN team
 * $Id$
 *
 * Author: Aurélien Chabot <aurelien at chabot dot fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

/*****************************************************************************
 * Preamble
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#ifdef HAVE_UNISTD_H
# include <unistd.h>
#endif

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_interface.h>
#include <vlc_playlist.h>
#include <vlc_input.h>
#include <vlc_aout.h>
#include <vlc_vout.h>
#include <vlc_keys.h>

#ifdef HAVE_POLL
# include <poll.h>
#endif

#include <upnp/upnp.h>
#include <upnp/upnptools.h>
#include <upnp/ixml.h>

/* Storage for xml file */
#define DEFAULT_WEB_DIR "/usr/share/vlc/upnp/"

/* XML upnp description file */
#define DEFAULT_DESC_FILE "vlc.xml"

/* String URL max size */
#define DESC_URL_SIZE 200

/* Advertissement validity time in second */
#define DEFAULT_ADVT_EXPIRE 100

/* String val len */
#define MAX_VAL_LEN 50

/* Service */
#define SERVICE_TYPE "urn:schemas-upnp-org:service-1-0"

#define MAX_VOLUME 2

/*****************************************************************************
 * Module descriptor
 *****************************************************************************/
static int  Open    ( vlc_object_t * );
static void Close   ( vlc_object_t * );

vlc_module_begin ()
    set_shortname( N_("UPNP") )
    set_category( CAT_INTERFACE )
    set_subcategory( SUBCAT_INTERFACE_CONTROL )
    set_description( N_("UPNP remote control interface") )
    set_capability( "interface", 0 )
    set_callbacks( Open, Close )
vlc_module_end ()

/*****************************************************************************
 * intf_sys_t: description and status of interface
 *****************************************************************************/
struct intf_sys_t
{
    vlc_mutex_t  lock;
    vlc_thread_t thread;
    int          i_fd;
};

/*****************************************************************************
 * Callback
 *****************************************************************************/

static int Callback(Upnp_EventType event_type, void *p_event, void *p_user_data);

static int HandleSubscriptionRequest(struct Upnp_Subscription_Request *event, intf_thread_t *p_intf);

static int HandleGetVarRequest(struct Upnp_State_Var_Request *event, intf_thread_t *p_intf);

static int HandleActionRequest(struct Upnp_Action_Request *event, intf_thread_t *p_intf);

/*****************************************************************************
 * Local variable
 *****************************************************************************/

UpnpDevice_Handle device_handle = -1;

/*****************************************************************************
 * Open: initialize interface
 *****************************************************************************/
static int Open( vlc_object_t *p_this )
{
    intf_thread_t *p_intf = (intf_thread_t *)p_this;
    int ret = UPNP_E_SUCCESS;
    char desc_doc_url[DESC_URL_SIZE];
    char *ip_address;
    unsigned short port;

    msg_Info(p_intf, "UPnP Initialization");

    /* Allocate instance and initialize some members */
    p_intf->p_sys = (intf_sys_t*) calloc( 1, sizeof( intf_sys_t ) );
    if( p_intf->p_sys == NULL )
        return VLC_ENOMEM;

    vlc_mutex_init( &p_intf->p_sys->lock );

    ret = UpnpInit( NULL, 0);
    if( ret != UPNP_E_SUCCESS )
    {
        msg_Err(p_intf, "Initialization failed: %s", UpnpGetErrorMessage( ret ) );
        goto error;
    }

    ip_address = UpnpGetServerIpAddress();
    port = UpnpGetServerPort();
    msg_Info(p_intf, "UPnP Initialized ipaddress = %s port = %u", ip_address ? ip_address : "NULL", port);

    msg_Info(p_intf, "Server root dir: %s", DEFAULT_WEB_DIR);
    ret = UpnpSetWebServerRootDir(DEFAULT_WEB_DIR);
    if (ret != UPNP_E_SUCCESS) {
        msg_Err(p_intf, "Error setting web root dir: %s", UpnpGetErrorMessage(ret));
        UpnpFinish();
        goto error;
    }

    snprintf(desc_doc_url, DESC_URL_SIZE, "http://%s:%d/%s", ip_address, port, DEFAULT_DESC_FILE);
    msg_Info(p_intf, "Server root device: %s", desc_doc_url);
    ret = UpnpRegisterRootDevice(
        desc_doc_url, Callback, p_intf, &device_handle);
    if (ret != UPNP_E_SUCCESS) {
        msg_Err(p_intf, "Error registering root device: %s", UpnpGetErrorMessage(ret));
        UpnpFinish();
        goto error;
    }

    ret = UpnpSendAdvertisement(device_handle, DEFAULT_ADVT_EXPIRE);
    if (ret != UPNP_E_SUCCESS) {
        msg_Err(p_intf, "Error sedding advertissement: %s", UpnpGetErrorMessage(ret));
        UpnpFinish();
        goto error;
    }

    msg_Info(p_intf, "UPnP Initialization done");

    return VLC_SUCCESS;

error:
    free( p_intf->p_sys );
    vlc_mutex_destroy(&p_intf->p_sys->lock);
    return VLC_EGENERIC;
}

/*****************************************************************************
 * Close: destroy interface
 *****************************************************************************/
static void Close( vlc_object_t *p_this )
{
    intf_thread_t *p_intf = (intf_thread_t *)p_this;
    intf_sys_t *p_sys = p_intf->p_sys;

    /* Destroy structure */
    vlc_mutex_destroy(&p_sys->lock);
    free( p_sys );
}

/*****************************************************************************
 * Utils
 *****************************************************************************/

void second2string(uint64_t seconds, char* time_c)
{
    uint64_t time_tot, time_h, time_m, time_s;

    time_h = (uint64_t) seconds / 3600;
    time_m = (uint64_t) ( seconds % 3600 ) / 60;
    time_s = (uint64_t) ( seconds % 60 );

    sprintf( time_c, "%02"PRIu64":""%02"PRIu64":""%02"PRIu64, time_h, time_m, time_s);
}

/*****************************************************************************
 * Handlers
 *****************************************************************************/

static int Callback( Upnp_EventType event_type, void *p_event, void *p_user_data )
{
    intf_thread_t *p_intf = (intf_thread_t *) p_user_data;
    vlc_mutex_lock(&p_intf->p_sys->lock);

    msg_Dbg( p_intf, "Callback event, type=%d", event_type );

    switch (event_type)
    {
        case UPNP_EVENT_SUBSCRIPTION_REQUEST:
        {
            struct Upnp_Subscription_Request* event = (struct Upnp_Subscription_Request *) p_event;
            msg_Dbg( p_intf, "Received subscription request");
            HandleSubscriptionRequest(event, p_intf);
        }
            break;
        case UPNP_CONTROL_GET_VAR_REQUEST:
        {
            struct Upnp_State_Var_Request* event = (struct Upnp_State_Var_Request *) p_event;
            msg_Dbg( p_intf, "Received var request: %s", event->StateVarName);
            HandleGetVarRequest(event, p_intf);
        }
            break;
        case UPNP_CONTROL_ACTION_REQUEST:
        {
            struct Upnp_Action_Request* event = (struct Upnp_Action_Request *) p_event;
            msg_Dbg( p_intf, "Received action request: %s", event->ActionName);
            HandleActionRequest(event, p_intf);
        }
            break;
        default:
            break;
    }

    vlc_mutex_unlock(&p_intf->p_sys->lock);
    return UPNP_E_SUCCESS;
}

int HandleSubscriptionRequest(struct Upnp_Subscription_Request *event, intf_thread_t *p_intf)
{
    return UPNP_E_SUCCESS;
}

int HandleGetVarRequest(struct Upnp_State_Var_Request *event, intf_thread_t *p_intf)
{
    // intf_sys_t *p_sys = p_intf->p_sys;
    // playlist_t *p_playlist = pl_Get( p_intf );

    event->ErrCode = UPNP_E_SUCCESS;

    if(!strcmp(event->StateVarName,"GetTransportInfo")){

    }
    else
    {
        event->ErrCode = UPNP_E_INTERNAL_ERROR;
    }

    return event->ErrCode;
}

int HandleActionRequest(struct Upnp_Action_Request *event, intf_thread_t *p_intf)
{
    int ret = UPNP_E_SUCCESS;

    intf_sys_t *p_sys = p_intf->p_sys;
    playlist_t *p_playlist = pl_Get( p_intf );

    /* Update the input */
    input_thread_t *p_input = playlist_CurrentInput( p_playlist );

    /* Update the vout */
    vout_thread_t *p_vout = p_input ? input_GetVout( p_input ) : NULL;

    event->ErrCode = UPNP_E_SUCCESS;

    if(!strcmp(event->ActionName, "Play"))
    {
        playlist_Play( p_playlist );
    }
    else if(!strcmp(event->ActionName, "Pause"))
    {
        playlist_Pause( p_playlist );
    }
    else if(!strcmp(event->ActionName, "Next"))
    {
        playlist_Next( p_playlist );
    }
    else if(!strcmp(event->ActionName, "Prev"))
    {
        playlist_Prev( p_playlist );
    }
    else if(!strcmp(event->ActionName, "Stop"))
    {
        playlist_Stop( p_playlist );
    }
    else if(!strcmp(event->ActionName, "Seek"))
    {
        msg_Info(p_intf, "%s", ixmlPrintDocument(event->ActionRequest));

        if( p_input != NULL )
        {
            const char* time_c;
            IXML_Node* node;

            node = ixmlNode_getFirstChild(ixmlNodeList_item(ixmlDocument_getElementsByTagName(event->ActionRequest, "Target"),0));
            time_c = ixmlNode_getNodeValue(node);
            mtime_t t = ((int64_t)atoi( time_c )) * CLOCK_FREQ;
            var_SetFloat( p_input, "position", t );
        }
    }
    else if(!strcmp(event->ActionName,"GetVolume"))
    {
        char value[MAX_VAL_LEN];

        sprintf(value, "%d", (int)((playlist_VolumeGet( p_playlist )/MAX_VOLUME)*100));
        msg_Info(p_intf, "Current volume is %s", value);
        UpnpAddToActionResponse(&event->ActionResult, event->ActionName, SERVICE_TYPE, "CurrentVolume", value);
        //sprintf(event->ActionResult, "%f", playlist_VolumeGet( p_playlist ));
    }
    else if(!strcmp(event->ActionName, "SetVolume"))
    {
        const char* volume_c;
        int volume;
        IXML_Node* node;

        //msg_Info(p_intf, "%s", ixmlPrintDocument(event->ActionRequest));

        node = ixmlNode_getFirstChild(ixmlNodeList_item(ixmlDocument_getElementsByTagName(event->ActionRequest, "DesiredVolume"),0));
        volume_c = ixmlNode_getNodeValue(node);

        if(volume_c)
        {
            msg_Info(p_intf, "Set volume to %s", volume_c);
            volume = atoi(volume_c);
            if( playlist_VolumeSet( p_playlist, ((float)volume*MAX_VOLUME)/100) != 0 )
            {
                msg_Err(p_intf, "Enable to set volume !!!");
                ret = UPNP_E_INTERNAL_ERROR;
            }
        }
        //DisplayVolume( p_intf, p_vout, volume / (float)AOUT_VOLUME_DEFAULT );
    }
    else if(!strcmp(event->ActionName, "SetMute"))
    {
        playlist_MuteToggle( p_playlist );
        /*if( playlist_MuteToggle( p_playlist ) == 0 )
        {
            float vol = playlist_VolumeGet( p_playlist );
            if( playlist_MuteGet( p_playlist ) > 0 || vol == 0.f )
            {
                ClearChannels( p_intf, p_vout );
                DisplayIcon( p_vout, OSD_MUTE_ICON );
            }
            else
                DisplayVolume( p_intf, p_vout, vol );
        }*/
    }
    else if(!strcmp(event->ActionName,"GetMediaInfo"))
    {
        msg_Info(p_intf, "%s", ixmlPrintDocument(event->ActionRequest));


    }
    else if(!strcmp(event->ActionName,"GetPositionInfo"))
    {
        msg_Info(p_intf, "%s", ixmlPrintDocument(event->ActionRequest));

        if( p_input != NULL )
        {
            vlc_value_t time;
            char time_c[8];
            uint64_t time_tot, time_h, time_m, time_s;
            var_Get( p_input, "length", &time );
            second2string(time.i_time / 1000000,  time_c);
            UpnpAddToActionResponse(&event->ActionResult, event->ActionName, SERVICE_TYPE, "TrackDuration", time_c);

            UpnpAddToActionResponse(&event->ActionResult, event->ActionName, SERVICE_TYPE, "Track", input_GetItem(p_input)->psz_name);

            var_Get( p_input, "time", &time );
            second2string(time.i_time / 1000000,  time_c);
            msg_Info(p_intf, "%"PRIu64" -> %s", time_tot, time_c);
            UpnpAddToActionResponse(&event->ActionResult, event->ActionName, SERVICE_TYPE, "RelTime", time_c);
            UpnpAddToActionResponse(&event->ActionResult, event->ActionName, SERVICE_TYPE, "AbsTime", time_c);

            msg_Info(p_intf, "%s", ixmlPrintDocument(event->ActionResult));
        }
    }
    else if(!strcmp(event->ActionName,"SetAVTransportURI"))
    {
        msg_Info(p_intf, "%s", ixmlPrintDocument(event->ActionRequest));
    }
    else
    {
        ret = UPNP_E_INTERNAL_ERROR;
    }

    return ret;
}
