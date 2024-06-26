# -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.71])
AC_INIT([sns],[1.1.0],[ndantam@mines.edu])
AM_INIT_AUTOMAKE([-Wall subdir-objects])
AC_CONFIG_SRCDIR([include/sns.h])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CXX
AC_PROG_CC
AM_PROG_AR
LT_INIT


AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

# Checks for header files.
dnl AC_CHECK_HEADERS([arpa/inet.h fcntl.h inttypes.h stdint.h stdlib.h string.h sys/socket.h syslog.h unistd.h time.h])

# Checks for typedefs, structures, and compiler characteristics.
dnl AC_HEADER_STDBOOL
dnl AC_TYPE_INT32_T
dnl AC_TYPE_MODE_T
dnl AC_TYPE_OFF_T
dnl AC_TYPE_PID_T
dnl AC_TYPE_SIZE_T
dnl AC_TYPE_SSIZE_T
dnl AC_TYPE_UINT32_T
dnl AC_TYPE_UINT64_T
dnl AC_TYPE_UINT8_T

dnl AC_CHECK_SIZEOF(void *)
dnl AC_CHECK_SIZEOF(uint64_t)
dnl AC_CHECK_SIZEOF(unsigned long)
dnl AC_CHECK_SIZEOF(unsigned long long)

# Check 32/64 safe format specifiers
dnl AC_CHECK_DECLS([PRIuPTR,PRIu64,PRIx64])

dnl AC_CHECK_FUNCS([ftruncate isascii memmove memset munmap socket strcasecmp strchr strdup strerror strtol])
PKG_CHECK_MODULES([AMINO], [amino])
AC_SUBST([AMINO_CFLAGS])
AC_SUBST([AMINO_LIBS])

PKG_CHECK_MODULES([ACH], [ach])
AC_SUBST([ACH_CFLAGS])
AC_SUBST([ACH_LIBS])

ACH_LIBS="$ACH_LIBS -lach-experimental"

CFLAGS="$CFLAGS $AMINO_CFLAGS $ACH_CFLAGS"




AC_SEARCH_LIBS([pthread_create],[pthread])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_SEARCH_LIBS([timer_settime],[rt])
AC_SEARCH_LIBS([dlopen],[dl])

AC_SEARCH_LIBS([dgemm_],[openblas blas])
AC_SEARCH_LIBS([dgesvd_],[lapack])

# Enable maximum warnings
m4_ifdef([AX_CFLAGS_WARN_ALL],
         [AX_CFLAGS_WARN_ALL])

dnl The AX_CHECK_COMPILER_FLAGS maco was renamed to AX_CHECK_COMPILE_FLAG
dnl Checking for both versions
m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])])

dnl If we found the flag checking macro, check some flags
m4_ifdef([APPEND_FLAG],
         [APPEND_FLAG([-Wextra])
          APPEND_FLAG([-Wconversion])
          APPEND_FLAG([-Wpointer-arith])
          APPEND_FLAG([-Wfloat-equal])
          APPEND_FLAG([-Wshadow])
          APPEND_FLAG([-Wwrite-strings])
          APPEND_FLAG([-Wc++-compat])])

dnl Fix the bad default for ar flags in automake
AR_FLAGS="cr"

## Doxygen ##
AC_ARG_WITH([doxygen],
            AS_HELP_STRING([--without-doxygen], [Disable doxygen support]),
            [],
            [with_doxygen=auto])
AS_IF([test "x$with_doxygen" != xno],
      [AC_CHECK_PROGS([DOXYGEN],[doxygen])])
AS_IF([test "x$with_doxygen" = xyes -a "x$DOXYGEN" = x],
      [AC_MSG_ERROR([Doxygen not found])])
AM_CONDITIONAL([HAVE_DOXYGEN], [test "x$DOXYGEN" != x])



AC_ARG_ENABLE([man],
              [AS_HELP_STRING([--disable-man], [Do not build man pages])],
              [],
              [enable_man="yes"])

AS_IF([test "x$enable_man" = xyes -a "x$build" = "x$host"],
      [AC_CHECK_PROGS([HELP2MAN], [help2man])])

AM_CONDITIONAL([HAVE_HELP2MAN], [test x$HELP2MAN != x])


## Amino ##
PKG_CHECK_MODULES([AMINO],
                  [amino],
                  [HAVE_AMINO=yes])
CFLAGS="$CFLAGS $AMINO_CFLAGS"
LDFLAGS="$LDFLAGS $AMINO_LDFLAGS"

## SDL ##
PKG_CHECK_MODULES([GL],
                  [gl],
                  [HAVE_GL=yes])
AC_SUBST([GL_CFLAGS])
AC_SUBST([GL_LIBS])

PKG_CHECK_MODULES([SDL2],
                  [sdl2],
                  [HAVE_SDL2=yes])

AC_SUBST([SDL2_CFLAGS])
AC_SUBST([SDL2_LIBS])

AC_CONFIG_FILES([Makefile share/sns.pc Doxyfile])

AC_OUTPUT


AS_BOX([SNS CONFIGURATION SUMMARY])
AC_MSG_NOTICE([prefix:            $prefix])
AC_MSG_NOTICE([C compiler:        $CC])
AC_MSG_NOTICE([C++ compiler:      $CXX])
AC_MSG_NOTICE([CFLAGS:            $CFLAGS])
AC_MSG_NOTICE([C++ FLAGS:         $CXXFLAGS])
