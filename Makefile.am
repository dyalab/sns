ACLOCAL_AMFLAGS = -Im4
AM_CPPFLAGS = -I$(top_srcdir)/include

EXTRA_DIST =
bin_PROGRAMS =
noinst_PROGRAMS =
bin_SCRIPTS =

include_HEADERS = include/sns.h
pkginclude_HEADERS =                   \
	include/sns/daemon.h	       \
	include/sns/event.h	       \
	include/sns/joystick/gamepad.h \
	include/sns/joystick/js.h      \
	include/sns/motor.h            \
	include/sns/msg.h              \
	include/sns/path.h             \
	include/sns/sdh_tactile.h      \
	include/sns/util.h        


## sns-daemon runner ##
EXTRA_DIST += scripts/sns

init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = scripts/sns

lib_LTLIBRARIES = libsns.la
libsns_la_SOURCES = src/msg.c src/daemon.c src/util.c src/msg/path.c src/event.c src/motor.c
libsns_la_LIBADD = $(AMINO_LIBS) $(ACH_LIBS)

## PLUGINS
lib_LTLIBRARIES += libsns_msg_joystick.la
libsns_msg_joystick_la_SOURCES = src/msg_plugin.c
libsns_msg_joystick_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_joystick
libsns_msg_joystick_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_motor_ref.la
libsns_msg_motor_ref_la_SOURCES = src/msg_plugin.c
libsns_msg_motor_ref_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_motor_ref
libsns_msg_motor_ref_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_tag_motor_ref.la
libsns_msg_tag_motor_ref_la_SOURCES = src/msg_plugin.c
libsns_msg_tag_motor_ref_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tag_motor_ref
libsns_msg_tag_motor_ref_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_motor_state.la
libsns_msg_motor_state_la_SOURCES = src/msg_plugin.c
libsns_msg_motor_state_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_motor_state
libsns_msg_motor_state_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_vector.la
libsns_msg_vector_la_SOURCES = src/msg_plugin.c
libsns_msg_vector_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_vector
libsns_msg_vector_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_tf.la
libsns_msg_tf_la_SOURCES = src/msg_plugin.c
libsns_msg_tf_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tf
libsns_msg_tf_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_wt_tf.la
libsns_msg_wt_tf_la_SOURCES = src/msg_plugin.c
libsns_msg_wt_tf_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_wt_tf
libsns_msg_wt_tf_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_tf_dx.la
libsns_msg_tf_dx_la_SOURCES = src/msg_plugin.c
libsns_msg_tf_dx_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tf_dx
libsns_msg_tf_dx_la_LIBADD = libsns.la


lib_LTLIBRARIES += libsns_msg_sdh_tactile.la
libsns_msg_sdh_tactile_la_SOURCES = src/msg_plugin.c
libsns_msg_sdh_tactile_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_sdh_tactile
libsns_msg_sdh_tactile_la_LIBADD = libsns.la

bin_PROGRAMS += snsdump
snsdump_SOURCES = src/snsdump.c
snsdump_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += snsplot
snsplot_SOURCES = src/snsplot.c
snsplot_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += sns-trylock
sns_trylock_SOURCES = src/sns-trylock.c

bin_PROGRAMS += snslogd
snslogd_SOURCES = src/snslogd.c
snslogd_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += snslog
snslog_SOURCES = src/snslog.c
snslog_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += snsref
snsref_SOURCES = src/snsref.c
snsref_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += snsrec
snsrec_SOURCES = src/snsrec.c
snsrec_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += snsreced
snsreced_SOURCES = src/snsreced.c
snsreced_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += snsmplex
snsmplex_SOURCES = src/bin/mplex.c
snsmplex_LDADD = libsns.la $(AMINO_LIBS) $(ACH_LIBS)

bin_PROGRAMS += sns-ksim
sns_ksim_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
sns_ksim_SOURCES = src/bin/sns-ksim.c
sns_ksim_LDADD = libsns.la -lach-experimental -lach -lamino -lamino-gl

bin_PROGRAMS += sns-teleopd
sns_teleopd_SOURCES = src/bin/sns-teleopd.c
sns_teleopd_LDADD = libsns.la -lach-experimental -lach -lamino


bin_PROGRAMS += sns-joyd
sns_joyd_SOURCES = src/driver/joystick/joyd.c src/driver/joystick/js.c
sns_joyd_LDADD = libsns.la -lach -lamino

noinst_PROGRAMS += demo/motor-ref
demo_motor_ref_SOURCES = demo/motor-ref.c
demo_motor_ref_LDADD = libsns.la -lach -lamino

noinst_PROGRAMS += demo/motor-state
demo_motor_state_SOURCES = demo/motor-state.c
demo_motor_state_LDADD = libsns.la -lach -lamino

noinst_PROGRAMS += demo/multi-ref-send
demo_multi_ref_send_SOURCES = demo/multi-ref-send.c
demo_multi_ref_send_LDADD = libsns.la -lach -lamino

noinst_PROGRAMS += demo/multi-state-recv
demo_multi_state_recv_SOURCES = demo/multi-state-recv.c
demo_multi_state_recv_LDADD = libsns.la -lach -lamino

## Joystick Helper Scripts ##
EXTRA_DIST += \
	scripts/sns-joyd-logitech-dual-action \
	scripts/sns-joyd-logitech-f310

bin_SCRIPTS += scripts/sns-joyd-logitech-f310 scripts/sns-joyd-logitech-dual-action

EXTRA_DIST +=                            \
	doc/md/acknowledgments.md.header \
	doc/md/command.md	         \
	doc/md/concepts.md               \
	doc/md/copying.md.header         \
	doc/md/daemon.md                 \
	doc/md/download.md               \
	doc/md/driver.md                 \
	doc/md/mainpage.md               \
	doc/md/tutorial-programming.md   \
	doc/md/tutorial.md


install-exec-hook:
	ln -sf $(DESTDIR)$(init_ddir)/sns$(EXEEXT) \
	       $(DESTDIR)$(bindir)/sns$(EXEEXT)

distclean-local:
	-rm -rf doc/manual                  \
		doc/md/acknowledgments.md   \
                doc/md/copying.md




doc/md/copying.md: $(top_srcdir)/COPYING $(top_srcdir)/doc/md/copying.md.header
	$(MKDIR_P) doc/md/
	cat $(top_srcdir)/doc/md/copying.md.header  \
	    $(top_srcdir)/COPYING \
	    > doc/md/copying.md


doc/md/acknowledgments.md: $(top_srcdir)/AUTHORS $(top_srcdir)/doc/md/acknowledgments.md.header
	$(MKDIR_P) doc/md/
	cat $(top_srcdir)/doc/md/acknowledgments.md.header  \
	    $(top_srcdir)/AUTHORS \
	    > doc/md/acknowledgments.md


if HAVE_DOXYGEN
doxygen: Doxyfile doc/md/copying.md doc/md/acknowledgments.md
	$(DOXYGEN)

doc/manual: doxygen

doc: doxygen
	$(MKDIR_P) doc/manual/doc

else

doc/manual:
	$(MKDIR_P) doc/manual

endif

if HAVE_HELP2MAN
dist_man_MANS =

dist_man_MANS += doc/sns.1
doc/sns.1: scripts/sns $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option=--version-man --no-info -n "SNS Management Script" $(top_srcdir)/scripts/sns -o $@

dist_man_MANS += doc/snsref.1
doc/snsref.1: $(top_builddir)/snsref $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Set motor reference" $(top_builddir)/$< -o $@

dist_man_MANS += doc/snsdump.1
doc/snsdump.1: $(top_builddir)/snsdump $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Display human-readable messages" $(top_builddir)/$< -o $@

dist_man_MANS += doc/snslogd.1
doc/snslogd.1: $(top_builddir)/snslogd $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "SNS Logging Daemon" $(top_builddir)/$< -o $@

dist_man_MANS += doc/sns-ksim.1
doc/sns-ksim.1: $(top_builddir)/sns-ksim $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Kinematic Simulator" $(top_builddir)/$< -o $@


dist_man_MANS += doc/sns-joyd.1
doc/sns-joyd.1: $(top_builddir)/sns-joyd $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) -l --no-info -n "Joystick Driver" $(top_builddir)/$< -o $@

dist_man_MANS += doc/sns-teleopd.1
doc/sns-teleopd.1: $(top_builddir)/sns-teleopd $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Teleoperation Daemon" $(top_builddir)/$< -o $@


html_mans =
html_imgs =

ROFFIT_PROC=roffit --bare < $< | sed -e 's/This HTML.*//' > $@

html_mans += doc/sns.html
doc/sns.html: doc/sns.1
	$(ROFFIT_PROC)

html_mans += doc/snsref.html
doc/snsref.html: doc/snsref.1
	$(ROFFIT_PROC)

html_mans += doc/snsdump.html
doc/snsdump.html: doc/snsdump.1
	$(ROFFIT_PROC)

html_mans += doc/snslogd.html
doc/snslogd.html: doc/snslogd.1
	$(ROFFIT_PROC)

html_mans += doc/sns-ksim.html
doc/sns-ksim.html: doc/sns-ksim.1
	$(ROFFIT_PROC)

html_mans += doc/sns-joyd.html
doc/sns-joyd.html: doc/sns-joyd.1
	$(ROFFIT_PROC)

html_mans += doc/sns-teleopd.html
doc/sns-teleopd.html: doc/sns-teleopd.1
	$(ROFFIT_PROC)

doc/daemon.md: doc/md/daemon.md $(html_mans)
	m4 --prefix-builtins $< > $@

doc/command.md: doc/md/command.md $(html_mans)
	m4 --prefix-builtins $< > $@

doc/driver.md: doc/md/driver.md $(html_mans)
	m4 --prefix-builtins $< > $@

doc/tutorial-programming.md: doc/md/tutorial-programming.md demo/motor-ref.c demo/motor-state.c
	m4 --prefix-builtins $< > $@


html_imgs += doc/manual/img/ref.png
doc/manual/img/ref.png: doc/tex/ref.tex doc/tex/sns.sty
	$(MKDIR_P) doc/manual/img
	TEXINPUTS=$(top_srcdir)/doc/tex:$(TEXINPUTS) pdflatex ref.tex
	pdfcrop ref.pdf
	gs -q -dNOPAUSE -dBATCH -sDEVICE=pngalpha -r300 -dEPSCrop -sOutputFile=$@ ref-crop.pdf

html_imgs += doc/manual/img/teleop.png
doc/manual/img/teleop.png: doc/tex/teleop.tex doc/tex/sns.sty
	$(MKDIR_P) doc/manual/img
	TEXINPUTS=$(top_srcdir)/doc/tex:$(TEXINPUTS) pdflatex teleop.tex
	pdfcrop teleop.pdf
	gs -q -dNOPAUSE -dBATCH -sDEVICE=pngalpha -r300 -dEPSCrop -sOutputFile=$@ teleop-crop.pdf

html_imgs += doc/manual/img/multi.png
doc/manual/img/multi.png: doc/tex/multi.tex doc/tex/sns.sty
	$(MKDIR_P) doc/manual/img
	TEXINPUTS=$(top_srcdir)/doc/tex:$(TEXINPUTS) pdflatex multi.tex
	pdfcrop multi.pdf
	gs -q -dNOPAUSE -dBATCH -sDEVICE=pngalpha -r300 -dEPSCrop -sOutputFile=$@ multi-crop.pdf

html_imgs += doc/manual/img/priority.png
doc/manual/img/priority.png: doc/tex/priority.tex doc/tex/sns.sty
	$(MKDIR_P) doc/manual/img
	TEXINPUTS=$(top_srcdir)/doc/tex:$(TEXINPUTS) pdflatex priority.tex
	pdfcrop priority.pdf
	gs -q -dNOPAUSE -dBATCH -sDEVICE=pngalpha -r300 -dEPSCrop -sOutputFile=$@ priority-crop.pdf


EXTRA_DIST += $(html_mans) $(html_imgs) doc/tex
doxygen: $(html_mans) $(html_imgs) doc/daemon.md doc/command.md doc/driver.md doc/tutorial-programming.md


endif



EXTRA_DIST += doc/manual
