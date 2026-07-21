#!/bin/sh
# NOTE: aclocal MUST run before autoheader. aclocal collects the third-party
# macros (m4/, notably AM_ICONV / AM_GNU_GETTEXT) into aclocal.m4; autoheader
# then needs those to emit their AC_DEFINE templates (e.g. ICONV_CONST) into
# config.h.in. Upstream ran autoheader first and got away with it only because
# it shipped a pre-generated configure/config.h.in; regenerating from tracked
# inputs (as we do) requires the correct order, or ICONV_CONST is left undefined
# and PSFLoader.cpp / vb.cpp / the debugger fail to compile.
set -e

#gettextize --force --copy --intl
aclocal -I m4
autoheader
autoconf -W syntax,cross
automake -a -c -f

rm -rf autom4te.cache
