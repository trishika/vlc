codecdir = $(pluginsdir)/codec
codec_LTLIBRARIES =

AM_CPPFLAGS += -I$(srcdir)/codec

libddummy_plugin_la_SOURCES = codec/ddummy.c
codec_LTLIBRARIES += libddummy_plugin.la

libedummy_plugin_la_SOURCES = codec/edummy.c
if ENABLE_SOUT
codec_LTLIBRARIES += libedummy_plugin.la
endif


### Audio codecs ###

liba52_plugin_la_SOURCES = codec/a52.c codec/a52.h
codec_LTLIBRARIES += liba52_plugin.la

libadpcm_plugin_la_SOURCES = codec/adpcm.c
codec_LTLIBRARIES += libadpcm_plugin.la

libaes3_plugin_la_SOURCES = codec/aes3.c
codec_LTLIBRARIES += libaes3_plugin.la

libaraw_plugin_la_SOURCES = codec/araw.c
libaraw_plugin_la_LIBADD = $(LIBM)
codec_LTLIBRARIES += libaraw_plugin.la

libdts_plugin_la_SOURCES = codec/dts.c codec/dts_header.c codec/dts_header.h
codec_LTLIBRARIES += libdts_plugin.la

libfaad_plugin_la_SOURCES = codec/faad.c
libfaad_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPPFLAGS_faad)
libfaad_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libfaad_plugin_la_LIBADD = $(LIBS_faad) $(LIBM)
EXTRA_LTLIBRARIES += libfaad_plugin.la
codec_LTLIBRARIES += $(LTLIBfaad)

libg711_plugin_la_SOURCES = codec/g711.c
codec_LTLIBRARIES += libg711_plugin.la

libfluidsynth_plugin_la_SOURCES = codec/fluidsynth.c
libfluidsynth_plugin_la_CFLAGS = $(AM_CFLAGS) $(FLUIDSYNTH_CFLAGS)
libfluidsynth_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libfluidsynth_plugin_la_LIBADD = $(FLUIDSYNTH_LIBS)
EXTRA_LTLIBRARIES += libfluidsynth_plugin.la
codec_LTLIBRARIES += $(LTLIBfluidsynth)
if HAVE_DARWIN
libfluidsynth_plugin_la_LDFLAGS += -Wl,-framework,CoreFoundation,-framework,CoreServices
endif

liblpcm_plugin_la_SOURCES = codec/lpcm.c
codec_LTLIBRARIES += liblpcm_plugin.la

libmpeg_audio_plugin_la_SOURCES = codec/mpeg_audio.c
codec_LTLIBRARIES += libmpeg_audio_plugin.la

libuleaddvaudio_plugin_la_SOURCES = codec/uleaddvaudio.c
codec_LTLIBRARIES += libuleaddvaudio_plugin.la

libwma_fixed_plugin_la_SOURCES = \
	codec/wmafixed/asf.h codec/wmafixed/bswap.h \
	codec/wmafixed/fft.c codec/wmafixed/fft.h \
	codec/wmafixed/mdct.c codec/wmafixed/mdct.h \
	codec/wmafixed/wma.c codec/wmafixed/wmadeci.c \
	codec/wmafixed/bitstream.c codec/wmafixed/bitstream.h \
	codec/wmafixed/wmadata.h codec/wmafixed/wmadec.h \
	codec/wmafixed/wmafixed.c codec/wmafixed/wmafixed.h
libwma_fixed_plugin_la_CPPFLAGS = $(AM_CPPFLAGS)
libwma_fixed_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
EXTRA_LTLIBRARIES += libwma_fixed_plugin.la
codec_LTLIBRARIES += $(LTLIBwma_fixed)


### Video codecs ###

libcdg_plugin_la_SOURCES = codec/cdg.c
codec_LTLIBRARIES += libcdg_plugin.la

liblibmpeg2_plugin_la_SOURCES = codec/libmpeg2.c
liblibmpeg2_plugin_la_CFLAGS = $(AM_CFLAGS) $(CFLAGS_libmpeg2)
liblibmpeg2_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
liblibmpeg2_plugin_la_LIBADD = $(LIBS_libmpeg2)
EXTRA_LTLIBRARIES += liblibmpeg2_plugin.la
codec_LTLIBRARIES += $(LTLIBlibmpeg2)

librawvideo_plugin_la_SOURCES = codec/rawvideo.c
codec_LTLIBRARIES += librawvideo_plugin.la

libschroedinger_plugin_la_SOURCES = codec/schroedinger.c
libschroedinger_plugin_la_CFLAGS = $(AM_CFLAGS) $(CFLAGS_schroedinger)
libschroedinger_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libschroedinger_plugin_la_LIBADD = $(LIBS_schroedinger)
EXTRA_LTLIBRARIES += libschroedinger_plugin.la
codec_LTLIBRARIES += $(LTLIBschroedinger)


### Image codecs ###

libpng_plugin_la_SOURCES = codec/png.c
libpng_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libpng_plugin_la_LIBADD = -lpng -lz $(LIBM)
EXTRA_LTLIBRARIES += libpng_plugin.la
codec_LTLIBRARIES += $(LTLIBpng)

libjpeg_plugin_la_SOURCES = codec/jpeg.c
libjpeg_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libjpeg_plugin_la_LIBADD = -ljpeg
EXTRA_LTLIBRARIES += libjpeg_plugin.la
codec_LTLIBRARIES += $(LTLIBjpeg)

libsvgdec_plugin_la_SOURCES = codec/svg.c
libsvgdec_plugin_la_CFLAGS = $(AM_CLAGS) $(CFLAGS_svgdec)
libsvgdec_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'  $(LDFLAGS_svg)
libsvgdec_plugin_la_LIBADD = $(LIBS_svgdec)
EXTRA_LTLIBRARIES += libsvgdec_plugin.la
codec_LTLIBRARIES += $(LTLIBsvgdec)

libsdl_image_plugin_la_SOURCES = codec/sdl_image.c
libsdl_image_plugin_la_CFLAGS = $(AM_CFLAGS) $(SDL_IMAGE_CFLAGS)
libsdl_image_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libsdl_image_plugin_la_LIBADD = $(SDL_IMAGE_LIBS)
EXTRA_LTLIBRARIES += libsdl_image_plugin.la
codec_LTLIBRARIES += $(LTLIBsdl_image)

libxwd_plugin_la_SOURCES = codec/xwd.c
libxwd_plugin_la_CFLAGS = $(AM_CFLAGS) $(XPROTO_CFLAGS)
libxwd_plugin_la_LIBADD = $(XPROTO_LIBS)
libxwd_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
if HAVE_XCB
codec_LTLIBRARIES += libxwd_plugin.la
endif


### SPU ###

liblibass_plugin_la_SOURCES = codec/libass.c
liblibass_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPPFLAGS_libass)
liblibass_plugin_la_CFLAGS = $(AM_CFLAGS) $(LIBASS_CFLAGS)
liblibass_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
liblibass_plugin_la_LIBADD = $(LIBASS_LIBS) $(LIBS_libass) $(LIBS_freetype)
EXTRA_LTLIBRARIES += liblibass_plugin.la
codec_LTLIBRARIES += $(LTLIBlibass)

libcc_plugin_la_SOURCES = codec/cc.c codec/cc.h codec/substext.h
codec_LTLIBRARIES += libcc_plugin.la

libcvdsub_plugin_la_SOURCES = codec/cvdsub.c
codec_LTLIBRARIES += libcvdsub_plugin.la

libdvbsub_plugin_la_SOURCES = codec/dvbsub.c
codec_LTLIBRARIES += libdvbsub_plugin.la

libscte27_plugin_la_SOURCES = codec/scte27.c
codec_LTLIBRARIES += libscte27_plugin.la

libspudec_plugin_la_SOURCES = codec/spudec/spudec.c codec/spudec/parse.c codec/spudec/spudec.h
libspudec_plugin_la_CPPFLAGS = $(AM_CPPFLAGS)
codec_LTLIBRARIES += libspudec_plugin.la

libstl_plugin_la_SOURCES = codec/stl.c
codec_LTLIBRARIES += libstl_plugin.la

libsubsdec_plugin_la_SOURCES = codec/subsdec.c codec/substext.h
codec_LTLIBRARIES += libsubsdec_plugin.la

libsubsusf_plugin_la_SOURCES = codec/subsusf.c
codec_LTLIBRARIES += libsubsusf_plugin.la

libsvcdsub_plugin_la_SOURCES = codec/svcdsub.c
codec_LTLIBRARIES += libsvcdsub_plugin.la

libt140_plugin_la_SOURCES = codec/t140.c
if ENABLE_SOUT
codec_LTLIBRARIES += libt140_plugin.la
endif

libtelx_plugin_la_SOURCES = codec/telx.c
libtelx_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
EXTRA_LTLIBRARIES += libtelx_plugin.la
codec_LTLIBRARIES += $(LTLIBtelx)

libzvbi_plugin_la_SOURCES = codec/zvbi.c
libzvbi_plugin_la_CFLAGS = $(AM_CFLAGS) $(ZVBI_CFLAGS) $(CFLAGS_zvbi)
libzvbi_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libzvbi_plugin_la_LIBADD = $(ZVBI_LIBS) $(SOCKET_LIBS)
if HAVE_WIN32
libzvbi_plugin_la_LIBADD += -lpthreadGC2
endif
EXTRA_LTLIBRARIES += libzvbi_plugin.la
codec_LTLIBRARIES += $(LTLIBzvbi)

libsubstx3g_plugin_la_SOURCES = codec/substx3g.c codec/substext.h
codec_LTLIBRARIES += libsubstx3g_plugin.la

### Xiph ###

libflac_plugin_la_SOURCES = codec/flac.c
libflac_plugin_la_CFLAGS = $(AM_CFLAGS) $(FLAC_CFLAGS)
libflac_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libflac_plugin_la_LIBADD = $(FLAC_LIBS)
EXTRA_LTLIBRARIES += libflac_plugin.la
codec_LTLIBRARIES += $(LTLIBflac)

libkate_plugin_la_SOURCES = codec/kate.c
libkate_plugin_la_CFLAGS = $(AM_CFLAGS) $(TIGER_CFLAGS) $(KATE_CFLAGS)
libkate_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libkate_plugin_la_LIBADD = $(TIGER_LIBS) $(KATE_LIBS) $(LIBM)
EXTRA_LTLIBRARIES += libkate_plugin.la
codec_LTLIBRARIES += $(LTLIBkate)

libopus_plugin_la_SOURCES = codec/opus.c codec/opus_header.c codec/opus_header.h
libopus_plugin_la_CFLAGS = $(AM_CFLAGS) $(OPUS_CFLAGS)
libopus_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libopus_plugin_la_LIBADD = $(OPUS_LIBS) $(LIBM)
EXTRA_LTLIBRARIES += libopus_plugin.la
codec_LTLIBRARIES += $(LTLIBopus)

libspeex_plugin_la_SOURCES = codec/speex.c
libspeex_plugin_la_CFLAGS = $(AM_CFLAGS) $(SPEEX_CFLAGS)
libspeex_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libspeex_plugin_la_LIBADD = $(SPEEX_LIBS) $(LIBM)
EXTRA_LTLIBRARIES += libspeex_plugin.la
codec_LTLIBRARIES += $(LTLIBspeex)

libtheora_plugin_la_SOURCES = codec/theora.c
libtheora_plugin_la_CFLAGS = $(AM_CFLAGS) $(THEORA_CFLAGS)
libtheora_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libtheora_plugin_la_LIBADD = $(THEORA_LIBS)
EXTRA_LTLIBRARIES += libtheora_plugin.la
codec_LTLIBRARIES += $(LTLIBtheora)

libtremor_plugin_la_SOURCES = codec/vorbis.c
libtremor_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -DMODULE_NAME_IS_tremor
libtremor_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libtremor_plugin_la_LIBADD = -lvorbisdec -logg
EXTRA_LTLIBRARIES += libtremor_plugin.la
codec_LTLIBRARIES += $(LTLIBtremor)

libvorbis_plugin_la_SOURCES = codec/vorbis.c
libvorbis_plugin_la_CFLAGS = $(AM_CFLAGS) $(VORBIS_CFLAGS)
libvorbis_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libvorbis_plugin_la_LIBADD = $(VORBIS_LIBS)
EXTRA_LTLIBRARIES += libvorbis_plugin.la
codec_LTLIBRARIES += $(LTLIBvorbis)


### FFmpeg/libav ###

libavcodec_plugin_la_SOURCES = \
	codec/avcodec/avcommon_compat.h \
	codec/avcodec/avcommon.h \
	codec/avcodec/video.c \
	codec/avcodec/subtitle.c \
	codec/avcodec/audio.c \
	codec/avcodec/fourcc.c \
	codec/avcodec/chroma.c codec/avcodec/chroma.h \
	codec/avcodec/va.c codec/avcodec/va.h \
	codec/avcodec/avcodec.c codec/avcodec/avcodec.h
if ENABLE_SOUT
libavcodec_plugin_la_SOURCES += codec/avcodec/encoder.c
endif
libavcodec_plugin_la_CFLAGS = $(AVCODEC_CFLAGS) $(AM_CFLAGS)
libavcodec_plugin_la_LIBADD = $(AVCODEC_LIBS) $(LIBM)
libavcodec_plugin_la_LDFLAGS = $(AM_LDFLAGS) $(SYMBOLIC_LDFLAGS)

if MERGE_FFMPEG
libavcodec_plugin_la_SOURCES += demux/avformat/demux.c access/avio.c
libavcodec_plugin_la_CFLAGS += $(AVFORMAT_CFLAGS)
libavcodec_plugin_la_LIBADD += $(AVFORMAT_LIBS)
if ENABLE_SOUT
libavcodec_plugin_la_SOURCES += demux/avformat/mux.c
endif
libavcodec_plugin_la_CFLAGS += -DMERGE_FFMPEG
endif
if HAVE_AVCODEC
codec_LTLIBRARIES += libavcodec_plugin.la
endif

### avcodec hardware acceleration ###

libvaapi_drm_plugin_la_SOURCES = \
	video_chroma/copy.c video_chroma/copy.h \
	codec/avcodec/vaapi.c
libvaapi_drm_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -DVLC_VA_BACKEND_DRM
libvaapi_drm_plugin_la_CFLAGS = $(AM_CFLAGS) \
	$(LIBVA_DRM_CFLAGS) $(AVCODEC_CFLAGS)
libvaapi_drm_plugin_la_LIBADD = $(LIBVA_DRM_LIBS)
libvaapi_x11_plugin_la_SOURCES = \
	video_chroma/copy.c video_chroma/copy.h \
	codec/avcodec/vaapi.c
libvaapi_x11_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -DVLC_VA_BACKEND_XLIB
libvaapi_x11_plugin_la_CFLAGS = $(AM_CFLAGS) \
	$(LIBVA_X11_CFLAGS) $(X_CFLAGS) $(AVCODEC_CFLAGS)
libvaapi_x11_plugin_la_LIBADD = $(LIBVA_X11_LIBS) $(X_LIBS) $(X_PRE_LIBS) -lX11
if HAVE_AVCODEC_VAAPI
if HAVE_VAAPI_DRM
codec_LTLIBRARIES += libvaapi_drm_plugin.la
endif
if HAVE_VAAPI_X11
codec_LTLIBRARIES += libvaapi_x11_plugin.la
endif
endif

libdxva2_plugin_la_SOURCES = \
	video_chroma/copy.c video_chroma/copy.h \
	codec/avcodec/dxva2.c
libdxva2_plugin_la_LIBADD = -lole32 -lshlwapi -luuid
if HAVE_AVCODEC_DXVA2
codec_LTLIBRARIES += libdxva2_plugin.la
endif

libvda_plugin_la_SOURCES = \
	video_chroma/copy.c video_chroma/copy.h \
	codec/avcodec/vda.c
libvda_plugin_la_CFLAGS = $(AM_CFLAGS) $(AVCODEC_CFLAGS)
libvda_plugin_la_LDFLAGS = -Wl,-framework,CoreFoundation,-framework,VideoDecodeAcceleration,-framework,QuartzCore
libvda_plugin_la_LIBADD = $(AVCODEC_LIBS)
if HAVE_AVCODEC_VDA
codec_LTLIBRARIES += libvda_plugin.la
endif

libhwdummy_plugin_la_SOURCES = codec/avcodec/hwdummy.c
libhwdummy_plugin_la_CFLAGS = $(AM_CFLAGS) $(VDPAU_CFLAGS) $(AVCODEC_CFLAGS)
libhwdummy_plugin_la_LIBADD = $(AVCODEC_LIBS)
# FIXME: hwdummy only needs libavutil, and only if avcodec < 55.13.0
if HAVE_AVCODEC_VDPAU
codec_LTLIBRARIES += libhwdummy_plugin.la
endif


### OpenMAX ###

noinst_HEADERS += \
	codec/omxil/OMX_Broadcom.h \
	codec/omxil/OMX_Component.h \
	codec/omxil/OMX_Core.h \
	codec/omxil/OMX_Image.h \
	codec/omxil/OMX_IVCommon.h \
	codec/omxil/OMX_Types.h \
	codec/omxil/OMX_Audio.h \
	codec/omxil/OMX_Index.h \
	codec/omxil/OMX_Other.h \
	codec/omxil/OMX_Video.h

libomxil_plugin_la_SOURCES = \
	codec/omxil/utils.c codec/omxil/omxil_utils.h \
	codec/h264_nal.h \
	codec/omxil/qcom.c codec/omxil/qcom.h \
	codec/omxil/omxil.c codec/omxil/omxil.h codec/omxil/omxil_core.c codec/omxil/omxil_core.h \
	video_chroma/copy.c
libomxil_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -I$(srcdir)/codec/omxil $(CFLAGS_omxil)
libomxil_plugin_la_LIBADD = $(LIBDL)
libomxil_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'

libomxil_vout_plugin_la_SOURCES = codec/omxil/vout.c codec/omxil/omxil_core.c codec/omxil/utils.c codec/omxil/qcom.c video_chroma/copy.c
libomxil_vout_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -I$(srcdir)/codec/omxil $(CFLAGS_omxil_vout)
libomxil_vout_plugin_la_LIBADD = $(LIBDL)
libomxil_vout_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'

libiomx_plugin_la_SOURCES = $(libomxil_plugin_la_SOURCES)
libiomx_plugin_la_CPPFLAGS = $(libomxil_plugin_la_CPPFLAGS) -DUSE_IOMX
libiomx_plugin_la_LIBADD = $(libomxil_plugin_la_LIBADD)

libmediacodec_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -I$(srcdir)/codec/omxil
libmediacodec_plugin_la_SOURCES = codec/omxil/android_mediacodec.c codec/omxil/utils.c \
	video_chroma/copy.c codec/omxil/android_opaque.c codec/omxil/android_opaque.h

codec_LTLIBRARIES += $(LTLIBomxil) $(LTLIBomxil_vout)
EXTRA_LTLIBRARIES += libomxil_plugin.la libomxil_vout_plugin.la
if HAVE_ANDROID
codec_LTLIBRARIES += libiomx_plugin.la libmediacodec_plugin.la
endif


### X26x encoders ###

libx265_plugin_la_SOURCES = codec/x265.c
libx265_plugin_la_CPPFLAGS = $(AM_CPPFLAGS)
libx265_plugin_la_CFLAGS = $(AM_CFLAGS) $(CFLAGS_x265)
libx265_plugin_la_LDFLAGS = $(AM_LDFLAGS) $(LDFLAGS_x265) -rpath '$(codecdir)'
libx265_plugin_la_LIBADD = $(LIBS_x265)
EXTRA_LTLIBRARIES += libx265_plugin.la
codec_LTLIBRARIES += $(LTLIBx265)

libx262_plugin_la_SOURCES = codec/x264.c
libx262_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -DMODULE_NAME_IS_x262
libx262_plugin_la_CFLAGS = $(AM_CFLAGS) $(CFLAGS_x262)
libx262_plugin_la_LDFLAGS = $(AM_LDFLAGS) $(LDFLAGS_x262) -rpath '$(codecdir)'
libx262_plugin_la_LIBADD = $(LIBS_x262)
EXTRA_LTLIBRARIES += libx262_plugin.la
codec_LTLIBRARIES += $(LTLIBx262)

libx264_plugin_la_SOURCES = codec/x264.c
libx264_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPPFLAGS_x264) -DMODULE_NAME_IS_x264
libx264_plugin_la_CFLAGS = $(AM_CFLAGS) $(CFLAGS_x264)
libx264_plugin_la_LDFLAGS = $(AM_LDFLAGS) $(LDFLAGS_x264) -rpath '$(codecdir)'
libx264_plugin_la_LIBADD = $(LIBS_x264)
EXTRA_LTLIBRARIES += libx264_plugin.la
codec_LTLIBRARIES += $(LTLIBx264)

libx26410b_plugin_la_SOURCES = codec/x264.c
libx26410b_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -DMODULE_NAME_IS_x26410b
libx26410b_plugin_la_CFLAGS = $(AM_CFLAGS) $(CFLAGS_x26410b)
libx26410b_plugin_la_LDFLAGS = $(AM_LDFLAGS) $(LDFLAGS_x26410b) -rpath '$(codecdir)'
libx26410b_plugin_la_LIBADD = $(LIBS_x26410b)
EXTRA_LTLIBRARIES += libx26410b_plugin.la
codec_LTLIBRARIES += $(LTLIBx26410b)


### Misc encoders ###

libfdkaac_plugin_la_SOURCES = codec/fdkaac.c
libfdkaac_plugin_la_CFLAGS = $(AM_CFLAGS) $(FDKAAC_CFLAGS)
libfdkaac_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libfdkaac_plugin_la_LIBADD = $(FDKAAC_LIBS)
EXTRA_LTLIBRARIES += libfdkaac_plugin.la
codec_LTLIBRARIES += $(LTLIBfdkaac)

libshine_plugin_la_SOURCES = codec/shine.c
libshine_plugin_la_CFLAGS = $(AM_CFLAGS) $(SHINE_CFLAGS)
libshine_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)' $(SHINE_LIBS)
EXTRA_LTLIBRARIES += libshine_plugin.la
codec_LTLIBRARIES += $(LTLIBshine)

libvpx_plugin_la_SOURCES = codec/vpx.c
libvpx_plugin_la_CPPFLAGS = $(AM_CPPFLAGS)
libvpx_plugin_la_CFLAGS = $(AM_CFLAGS) $(VPX_CFLAGS) $(CPPFLAGS_vpx)
libvpx_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libvpx_plugin_la_LIBADD = $(VPX_LIBS)
EXTRA_LTLIBRARIES += libvpx_plugin.la
codec_LTLIBRARIES += $(LTLIBvpx)

libtwolame_plugin_la_SOURCES = codec/twolame.c
libtwolame_plugin_la_CPPFLAGS = $(AM_CPPFLAGS) -DLIBTWOLAME_STATIC
libtwolame_plugin_la_CFLAGS = $(AM_CFLAGS) $(TWOLAME_CFLAGS)
libtwolame_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libtwolame_plugin_la_LIBADD = $(TWOLAME_LIBS) $(LIBM)
EXTRA_LTLIBRARIES += libtwolame_plugin.la
codec_LTLIBRARIES += $(LTLIBtwolame)


### Hardware encoders ###

libcrystalhd_plugin_la_SOURCES = codec/crystalhd.c codec/h264_nal.h
libcrystalhd_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libcrystalhd_plugin_la_LIBADD = $(LIBS_crystalhd)
EXTRA_LTLIBRARIES += libcrystalhd_plugin.la
codec_LTLIBRARIES += $(LTLIBcrystalhd)

libqsv_plugin_la_SOURCES = codec/qsv.c
libqsv_plugin_la_CFLAGS = $(AM_CFLAGS) $(MFX_CFLAGS)
libqsv_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libqsv_plugin_la_LIBADD = $(MFX_LIBS)
EXTRA_LTLIBRARIES += libqsv_plugin.la
codec_LTLIBRARIES += $(LTLIBqsv)


### External frameworks ###

libdmo_plugin_la_SOURCES = codec/dmo/dmo.c codec/dmo/dmo.h codec/dmo/buffer.c
libqsv_plugin_la_CPPFLAGS = $(AM_CPPFLAGS)
libdmo_plugin_la_LIBADD = $(LIBM)
if HAVE_WIN32
libdmo_plugin_la_LIBADD += -lole32 -luuid
codec_LTLIBRARIES += libdmo_plugin.la
endif

libmft_plugin_la_SOURCES = codec/mft.c
if HAVE_WIN32
libmft_plugin_la_LIBADD = -lole32 -luuid
codec_LTLIBRARIES += libmft_plugin.la
endif

libquicktime_plugin_la_SOURCES = codec/quicktime.c
libquicktime_plugin_la_LDFLAGS = $(AM_LDFLAGS) -rpath '$(codecdir)'
libquicktime_plugin_la_LIBADD = $(LIBM)
EXTRA_LTLIBRARIES += libquicktime_plugin.la
codec_LTLIBRARIES += $(LTLIBquicktime)

libgstdecode_plugin_la_SOURCES = codec/gstdecode.c
libgstdecode_plugin_la_CFLAGS = $(AM_CFLAGS) $(GST_VIDEO_CFLAGS) $(GST_APP_CFLAGS)
libgstdecode_plugin_la_LIBADD = $(GST_VIDEO_LIBS) $(GST_APP_LIBS)
if HAVE_GST_DECODE
codec_LTLIBRARIES += libgstdecode_plugin.la
endif
