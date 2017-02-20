
#--------------------------------------------------------
#= Begin: BOOST LIB -------------------------------------

INCLUDEPATH += $$(DEPENDENCIES_INCLUDE)
INCLUDEPATH += $$(DEPENDENCIES_INCLUDE)/boost

win32:CONFIG(release, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/boost -llibboost_wserialization-vc120-mt-1_57\
    -llibboost_timer-vc120-mt-1_57 \
    -llibboost_thread-vc120-mt-1_57 \
    -llibboost_system-vc120-mt-1_57 \
    -llibboost_signals-vc120-mt-1_57 \
    -llibboost_serialization-vc120-mt-1_57 \
    -llibboost_regex-vc120-mt-1_57 \
    -llibboost_random-vc120-mt-1_57 \
    -llibboost_program_options-vc120-mt-1_57 \
    -llibboost_prg_exec_monitor-vc120-mt-1_57 \
    -llibboost_mpi-vc120-mt-1_57 \
    -llibboost_math_tr1l-vc120-mt-1_57 \
    -llibboost_math_tr1f-vc120-mt-1_57 \
    -llibboost_math_tr1-vc120-mt-1_57 \
    -llibboost_math_c99l-vc120-mt-1_57 \
    -llibboost_math_c99f-vc120-mt-1_57 \
    -llibboost_math_c99-vc120-mt-1_57 \
    -llibboost_log_setup-vc120-mt-1_57 \
    -llibboost_log-vc120-mt-1_57 \
    -llibboost_locale-vc120-mt-1_57 \
    -llibboost_iostreams-vc120-mt-1_57 \
    -llibboost_graph-vc120-mt-1_57 \
    -llibboost_filesystem-vc120-mt-1_57 \
    -llibboost_exception-vc120-mt-1_57 \
    -llibboost_date_time-vc120-mt-1_57 \
    -llibboost_coroutine-vc120-mt-1_57 \
    -llibboost_context-vc120-mt-1_57 \
    -llibboost_container-vc120-mt-1_57 \
    -llibboost_chrono-vc120-mt-1_57 \
    -llibboost_atomic-vc120-mt-1_57

else:win32:CONFIG(debug, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/boost -llibboost_wserialization-vc120-mt-gd-1_57\
    -llibboost_timer-vc120-mt-gd-1_57 \
    -llibboost_thread-vc120-mt-gd-1_57 \
    -llibboost_system-vc120-mt-gd-1_57 \
    -llibboost_signals-vc120-mt-gd-1_57 \
    -llibboost_serialization-vc120-mt-gd-1_57 \
    -llibboost_regex-vc120-mt-gd-1_57 \
    -llibboost_random-vc120-mt-gd-1_57 \
    -llibboost_program_options-vc120-mt-gd-1_57 \
    -llibboost_prg_exec_monitor-vc120-mt-gd-1_57 \
    -llibboost_mpi-vc120-mt-gd-1_57 \
    -llibboost_math_tr1l-vc120-mt-gd-1_57 \
    -llibboost_math_tr1f-vc120-mt-gd-1_57 \
    -llibboost_math_tr1-vc120-mt-gd-1_57 \
    -llibboost_math_c99l-vc120-mt-gd-1_57 \
    -llibboost_math_c99f-vc120-mt-gd-1_57 \
    -llibboost_math_c99-vc120-mt-gd-1_57 \
    -llibboost_log_setup-vc120-mt-gd-1_57 \
    -llibboost_log-vc120-mt-gd-1_57 \
    -llibboost_locale-vc120-mt-gd-1_57 \
    -llibboost_iostreams-vc120-mt-gd-1_57 \
    -llibboost_graph-vc120-mt-gd-1_57 \
    -llibboost_filesystem-vc120-mt-gd-1_57 \
    -llibboost_exception-vc120-mt-gd-1_57 \
    -llibboost_date_time-vc120-mt-gd-1_57 \
    -llibboost_coroutine-vc120-mt-gd-1_57 \
    -llibboost_context-vc120-mt-gd-1_57 \
    -llibboost_container-vc120-mt-gd-1_57 \
    -llibboost_chrono-vc120-mt-gd-1_57 \
    -llibboost_atomic-vc120-mt-gd-1_57
#=   End: BOOST LIB -------------------------------------

