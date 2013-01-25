#-------------------------------------------------
#
# Project created by QtCreator 2012-05-15T18:29:46
#
#-------------------------------------------------

QT       += core gui

TARGET = HRM
TEMPLATE = app

FORMS    += mainwindow.ui

INCLUDEPATH += ../mxsrclib/
INCLUDEPATH += ../mxsrclib/arch/qt/
INCLUDEPATH += ../general/

QMAKE_CXXFLAGS += -finput-charset=CP1251
QMAKE_CXXFLAGS += -Wno-deprecated
QMAKE_CXXFLAGS += -Wextra
QMAKE_CXXFLAGS += -Wctor-dtor-privacy
QMAKE_CXXFLAGS += -Woverloaded-virtual

LIBS += -lws2_32


SOURCES += main.cpp\
        mainwindow.cpp \
    ../mxsrclib/niusbgpib_hardflow.cpp \
    ../mxsrclib/niusbgpib.cpp \
    ../mxsrclib/mxnetr.cpp \
    ../mxsrclib/mxnetc.cpp \
    ../mxsrclib/mxnet.cpp \
    ../mxsrclib/mxifar.cpp \
    ../mxsrclib/mxextchart.cpp \
    ../mxsrclib/mxextbase.cpp \
    ../mxsrclib/mxdata.cpp \
    ../mxsrclib/meassup.cpp \
    ../mxsrclib/measmul.cpp \
    ../mxsrclib/irsvariant.cpp \
    ../mxsrclib/irstime.cpp \
    ../mxsrclib/irstcpip.cpp \
    ../mxsrclib/irsstd.cpp \
    ../mxsrclib/irsprop.cpp \
    ../mxsrclib/irsmenu.cpp \
    ../mxsrclib/irsmem.cpp \
    ../mxsrclib/irsmcutil.cpp \
    ../mxsrclib/irsmbus.cpp \
    ../mxsrclib/irsmatrix.cpp \
    ../mxsrclib/irsmath.cpp \
    ../mxsrclib/irslocale.cpp \
    ../mxsrclib/irskbd.cpp \
    ../mxsrclib/irsint.cpp \
    ../mxsrclib/irsgpio.cpp \
    ../mxsrclib/irsfunnel.cpp \
    ../mxsrclib/irsfilter.cpp \
    ../mxsrclib/irsexcept.cpp \
    ../mxsrclib/irserror.cpp \
    ../mxsrclib/irsdsp.cpp \
    ../mxsrclib/irsdev.cpp \
    ../mxsrclib/irsdbgutil.cpp \
    ../mxsrclib/irscpu.cpp \
    ../mxsrclib/irscalc.cpp \
    ../mxsrclib/irsalg.cpp \
    ../mxsrclib/irsadc.cpp \
    ../mxsrclib/hardflowg.cpp \
    ../mxsrclib/dbgprn.cpp \
    ../mxsrclib/csvwork.cpp \
    ../mxsrclib/correct_alg.cpp \
    ../mxsrclib/timer.cpp \
    ../mxsrclib/arch/qt/mxifa.cpp \
    ../mxsrclib/arch/qt/counter.cpp \
    cfg.cpp \
    app.cpp \
    ../mxsrclib/irsstring.cpp \
    process.cpp \
    report.cpp

HEADERS  += mainwindow.h \
    ../mxsrclib/timer.h \
    ../mxsrclib/niusbgpib_hardflow.h \
    ../mxsrclib/niusbgpib.h \
    ../mxsrclib/mxnetr.h \
    ../mxsrclib/mxnetd.h \
    ../mxsrclib/mxnetc.h \
    ../mxsrclib/mxnet.h \
    ../mxsrclib/mxifar.h \
    ../mxsrclib/mxifa.h \
    ../mxsrclib/mxextchart.h \
    ../mxsrclib/mxextbase.h \
    ../mxsrclib/mxdatastd.h \
    ../mxsrclib/mxdata.h \
    ../mxsrclib/meassup.h \
    ../mxsrclib/measmul.h \
    ../mxsrclib/measdef.h \
    ../mxsrclib/irsvariant.h \
    ../mxsrclib/irstree.h \
    ../mxsrclib/irstime.h \
    ../mxsrclib/irstest.h \
    ../mxsrclib/irstcpip.h \
    ../mxsrclib/irstable.h \
    ../mxsrclib/irssysutils.h \
    ../mxsrclib/irsstrmstd.h \
    ../mxsrclib/irsstrm.h \
    ../mxsrclib/irsstring.h \
    ../mxsrclib/irsstrdefs.h \
    ../mxsrclib/irsstrconv.h \
    ../mxsrclib/irsstd.h \
    ../mxsrclib/irsspi.h \
    ../mxsrclib/irsrect.h \
    ../mxsrclib/irsprop.h \
    ../mxsrclib/irspch.h \
    ../mxsrclib/irsparamabs.h \
    ../mxsrclib/irsnetdefs.h \
    ../mxsrclib/irsmenu.h \
    ../mxsrclib/irsmem.h \
    ../mxsrclib/irsmcutil.h \
    ../mxsrclib/irsmbus.h \
    ../mxsrclib/irsmatrix.h \
    ../mxsrclib/irsmath.h \
    ../mxsrclib/irslocale.h \
    ../mxsrclib/irslimits.h \
    ../mxsrclib/irskbd.h \
    ../mxsrclib/irsint.h \
    ../mxsrclib/irsgpio.h \
    ../mxsrclib/irsfunnel.h \
    ../mxsrclib/irsfinal.h \
    ../mxsrclib/irsfilter.h \
    ../mxsrclib/irsexcept.h \
    ../mxsrclib/irserror.h \
    ../mxsrclib/irsdsp.h \
    ../mxsrclib/irsdev.h \
    ../mxsrclib/irsdefs.h \
    ../mxsrclib/irsdbgutil.h \
    ../mxsrclib/irscpu.h \
    ../mxsrclib/irscpp.h \
    ../mxsrclib/irsconsolestd.h \
    ../mxsrclib/irsconsole.h \
    ../mxsrclib/irsconfig.h \
    ../mxsrclib/irsconfig0.h \
    ../mxsrclib/irschartwin.h \
    ../mxsrclib/irscalc.h \
    ../mxsrclib/irsalg.h \
    ../mxsrclib/irsadcabs.h \
    ../mxsrclib/irsadc.h \
    ../mxsrclib/hardflowg.h \
    ../mxsrclib/dbgprn.h \
    ../mxsrclib/csvwork.h \
    ../mxsrclib/correct_alg.h \
    ../mxsrclib/tstlan4abs.h \
    ../mxsrclib/arch/qt/ni488.h \
    ../mxsrclib/arch/qt/mxnetda.h \
    ../mxsrclib/arch/qt/mxifal.h \
    ../mxsrclib/arch/qt/Decl-32.h \
    ../mxsrclib/arch/qt/counter.h \
    ../general/hrm_tbi_data.h \
    cfg.h \
    app.h \
    process.h \
    report.h \
    ../general/hrm_defs.h

