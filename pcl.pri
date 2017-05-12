INCLUDEPATH += $$(DEPENDENCIES_INCLUDE)

# BOOST
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

# COVER ALL GROUND
INCLUDEPATH += $$(DEPENDENCIES_INCLUDE)/

# FLANN
INCLUDEPATH += $$(DEPENDENCIES_LIB64)/flann
DEPENDPATH  += $$(DEPENDENCIES_LIB64)/flann

win32:CONFIG(debug, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/flann -lflann-gd \
        -lflann_s-gd \
        -lflann_cpp_s-gd

else:win32:CONFIG(release, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/flann -lflann \
        -lflann_s \
        -lflann_cpp_s

# VTK
INCLUDEPATH += $$(DEPENDENCIES_INCLUDE)/vtk
INCLUDEPATH +=  $$(DEPENDENCIES_LIB64)/vtk
DEPENDPATH  +=  $$(DEPENDENCIES_LIB64)/vtk

win32:CONFIG(release, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/vtk -lvtkViewsQt-6.1 \
        -lvtkalglib-6.1 \
        -lvtkChartsCore-6.1 \
        -lvtkCommonColor-6.1 \
        -lvtkCommonComputationalGeometry-6.1 \
        -lvtkCommonCore-6.1 \
        -lvtkCommonDataModel-6.1 \
        -lvtkCommonExecutionModel-6.1 \
        -lvtkCommonMath-6.1 \
        -lvtkCommonMisc-6.1 \
        -lvtkCommonSystem-6.1 \
        -lvtkCommonTransforms-6.1 \
        -lvtkDICOMParser-6.1 \
        -lvtkDomainsChemistry-6.1 \
        -lvtkexoIIc-6.1 \
        -lvtkexpat-6.1 \
        -lvtkFiltersAMR-6.1 \
        -lvtkFiltersCore-6.1 \
        -lvtkFiltersExtraction-6.1 \
        -lvtkFiltersFlowPaths-6.1 \
        -lvtkFiltersGeneral-6.1 \
        -lvtkFiltersGeneric-6.1 \
        -lvtkFiltersGeometry-6.1 \
        -lvtkFiltersHybrid-6.1 \
        -lvtkFiltersHyperTree-6.1 \
        -lvtkFiltersImaging-6.1 \
        -lvtkFiltersModeling-6.1 \
        -lvtkFiltersParallel-6.1 \
        -lvtkFiltersParallelImaging-6.1 \
        -lvtkFiltersProgrammable-6.1 \
        -lvtkFiltersSelection-6.1 \
        -lvtkFiltersSMP-6.1 \
        -lvtkFiltersSources-6.1 \
        -lvtkFiltersStatistics-6.1 \
        -lvtkFiltersTexture-6.1 \
        -lvtkFiltersVerdict-6.1 \
        -lvtkfreetype-6.1 \
        -lvtkftgl-6.1 \
        -lvtkGeovisCore-6.1 \
        -lvtkgl2ps-6.1 \
        -lvtkGUISupportQt-6.1 \
        -lvtkGUISupportQtOpenGL-6.1 \
        -lvtkGUISupportQtSQL-6.1 \
        -lvtkGUISupportQtWebkit-6.1 \
        -lvtkhdf5-6.1 \
        -lvtkhdf5_hl-6.1 \
        -lvtkImagingColor-6.1 \
        -lvtkImagingCore-6.1 \
        -lvtkImagingFourier-6.1 \
        -lvtkImagingGeneral-6.1 \
        -lvtkImagingHybrid-6.1 \
        -lvtkImagingMath-6.1 \
        -lvtkImagingMorphological-6.1 \
        -lvtkImagingSources-6.1 \
        -lvtkImagingStatistics-6.1 \
        -lvtkImagingStencil-6.1 \
        -lvtkInfovisCore-6.1 \
        -lvtkInfovisLayout-6.1 \
        -lvtkInteractionImage-6.1 \
        -lvtkInteractionStyle-6.1 \
        -lvtkInteractionWidgets-6.1 \
        -lvtkIOAMR-6.1 \
        -lvtkIOCore-6.1 \
        -lvtkIOEnSight-6.1 \
        -lvtkIOExodus-6.1 \
        -lvtkIOExport-6.1 \
        -lvtkIOGeometry-6.1 \
        -lvtkIOImage-6.1 \
        -lvtkIOImport-6.1 \
        -lvtkIOInfovis-6.1 \
        -lvtkIOLegacy-6.1 \
        -lvtkIOLSDyna-6.1 \
        -lvtkIOMINC-6.1 \
        -lvtkIOMovie-6.1 \
        -lvtkIONetCDF-6.1 \
        -lvtkIOParallel-6.1 \
        -lvtkIOPLY-6.1 \
        -lvtkIOSQL-6.1 \
        -lvtkIOVideo-6.1 \
        -lvtkIOXML-6.1 \
        -lvtkIOXMLParser-6.1 \
        -lvtkjpeg-6.1 \
        -lvtkjsoncpp-6.1 \
        -lvtklibxml2-6.1 \
        -lvtkmetaio-6.1 \
        -lvtkNetCDF-6.1 \
        -lvtkNetCDF_cxx-6.1 \
        -lvtkoggtheora-6.1 \
        -lvtkParallelCore-6.1 \
        -lvtkpng-6.1 \
        -lvtkproj4-6.1 \
        -lvtkPythonInterpreter-6.1 \
        -lvtkRenderingAnnotation-6.1 \
        -lvtkRenderingContext2D-6.1 \
        -lvtkRenderingCore-6.1 \
        -lvtkRenderingFreeType-6.1 \
        -lvtkRenderingFreeTypeOpenGL-6.1 \
        -lvtkRenderingGL2PS-6.1 \
        -lvtkRenderingImage-6.1 \
        -lvtkRenderingLabel-6.1 \
        -lvtkRenderingLIC-6.1 \
        -lvtkRenderingLOD-6.1 \
        -lvtkRenderingMatplotlib-6.1 \
        -lvtkRenderingOpenGL-6.1 \
        -lvtkRenderingParallel-6.1 \
        -lvtkRenderingQt-6.1 \
        -lvtkRenderingVolume-6.1 \
        -lvtkRenderingVolumeAMR-6.1 \
        -lvtkRenderingVolumeOpenGL-6.1 \
        -lvtksqlite-6.1 \
        -lvtksys-6.1 \
        -lvtktiff-6.1 \
        -lvtkverdict-6.1 \
        -lvtkViewsContext2D-6.1 \
        -lvtkViewsCore-6.1 \
        -lvtkViewsGeovis-6.1 \
        -lvtkViewsInfovis-6.1 \
        -lvtkWrappingTools-6.1 \
        -lvtkzlib-6.1

else:win32:CONFIG(debug, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/vtk -lvtkViewsQt-6.1-gd \
        -lvtkalglib-6.1-gd \
        -lvtkChartsCore-6.1-gd \
        -lvtkCommonColor-6.1-gd \
        -lvtkCommonComputationalGeometry-6.1-gd \
        -lvtkCommonCore-6.1-gd \
        -lvtkCommonDataModel-6.1-gd \
        -lvtkCommonExecutionModel-6.1-gd \
        -lvtkCommonMath-6.1-gd \
        -lvtkCommonMisc-6.1-gd \
        -lvtkCommonSystem-6.1-gd \
        -lvtkCommonTransforms-6.1-gd \
        -lvtkDICOMParser-6.1-gd \
        -lvtkDomainsChemistry-6.1-gd \
        -lvtkexoIIc-6.1-gd \
        -lvtkexpat-6.1-gd \
        -lvtkFiltersAMR-6.1-gd \
        -lvtkFiltersCore-6.1-gd \
        -lvtkFiltersExtraction-6.1-gd \
        -lvtkFiltersFlowPaths-6.1-gd \
        -lvtkFiltersGeneral-6.1-gd \
        -lvtkFiltersGeneric-6.1-gd \
        -lvtkFiltersGeometry-6.1-gd \
        -lvtkFiltersHybrid-6.1-gd \
        -lvtkFiltersHyperTree-6.1-gd \
        -lvtkFiltersImaging-6.1-gd \
        -lvtkFiltersModeling-6.1-gd \
        -lvtkFiltersParallel-6.1-gd \
        -lvtkFiltersParallelImaging-6.1-gd \
        -lvtkFiltersProgrammable-6.1-gd \
        -lvtkFiltersSelection-6.1-gd \
        -lvtkFiltersSMP-6.1-gd \
        -lvtkFiltersSources-6.1-gd \
        -lvtkFiltersStatistics-6.1-gd \
        -lvtkFiltersTexture-6.1-gd \
        -lvtkFiltersVerdict-6.1-gd \
        -lvtkfreetype-6.1-gd \
        -lvtkftgl-6.1-gd \
        -lvtkGeovisCore-6.1-gd \
        -lvtkgl2ps-6.1-gd \
        -lvtkGUISupportQt-6.1-gd \
        -lvtkGUISupportQtOpenGL-6.1-gd \
        -lvtkGUISupportQtSQL-6.1-gd \
        -lvtkGUISupportQtWebkit-6.1-gd \
        -lvtkhdf5-6.1-gd \
        -lvtkhdf5_hl-6.1-gd \
        -lvtkImagingColor-6.1-gd \
        -lvtkImagingCore-6.1-gd \
        -lvtkImagingFourier-6.1-gd \
        -lvtkImagingGeneral-6.1-gd \
        -lvtkImagingHybrid-6.1-gd \
        -lvtkImagingMath-6.1-gd \
        -lvtkImagingMorphological-6.1-gd \
        -lvtkImagingSources-6.1-gd \
        -lvtkImagingStatistics-6.1-gd \
        -lvtkImagingStencil-6.1-gd \
        -lvtkInfovisCore-6.1-gd \
        -lvtkInfovisLayout-6.1-gd \
        -lvtkInteractionImage-6.1-gd \
        -lvtkInteractionStyle-6.1-gd \
        -lvtkInteractionWidgets-6.1-gd \
        -lvtkIOAMR-6.1-gd \
        -lvtkIOCore-6.1-gd \
        -lvtkIOEnSight-6.1-gd \
        -lvtkIOExodus-6.1-gd \
        -lvtkIOExport-6.1-gd \
        -lvtkIOGeometry-6.1-gd \
        -lvtkIOImage-6.1-gd \
        -lvtkIOImport-6.1-gd \
        -lvtkIOInfovis-6.1-gd \
        -lvtkIOLegacy-6.1-gd \
        -lvtkIOLSDyna-6.1-gd \
        -lvtkIOMINC-6.1-gd \
        -lvtkIOMovie-6.1-gd \
        -lvtkIONetCDF-6.1-gd \
        -lvtkIOParallel-6.1-gd \
        -lvtkIOPLY-6.1-gd \
        -lvtkIOSQL-6.1-gd \
        -lvtkIOVideo-6.1-gd \
        -lvtkIOXML-6.1-gd \
        -lvtkIOXMLParser-6.1-gd \
        -lvtkjpeg-6.1-gd \
        -lvtkjsoncpp-6.1-gd \
        -lvtklibxml2-6.1-gd \
        -lvtkmetaio-6.1-gd \
        -lvtkNetCDF-6.1-gd \
        -lvtkNetCDF_cxx-6.1-gd \
        -lvtkoggtheora-6.1-gd \
        -lvtkParallelCore-6.1-gd \
        -lvtkpng-6.1-gd \
        -lvtkproj4-6.1-gd \
        -lvtkPythonInterpreter-6.1-gd \
        -lvtkRenderingAnnotation-6.1-gd \
        -lvtkRenderingContext2D-6.1-gd \
        -lvtkRenderingCore-6.1-gd \
        -lvtkRenderingFreeType-6.1-gd \
        -lvtkRenderingFreeTypeOpenGL-6.1-gd \
        -lvtkRenderingGL2PS-6.1-gd \
        -lvtkRenderingImage-6.1-gd \
        -lvtkRenderingLabel-6.1-gd \
        -lvtkRenderingLIC-6.1-gd \
        -lvtkRenderingLOD-6.1-gd \
        -lvtkRenderingMatplotlib-6.1-gd \
        -lvtkRenderingOpenGL-6.1-gd \
        -lvtkRenderingParallel-6.1-gd \
        -lvtkRenderingQt-6.1-gd \
        -lvtkRenderingVolume-6.1-gd \
        -lvtkRenderingVolumeAMR-6.1-gd \
        -lvtkRenderingVolumeOpenGL-6.1-gd \
        -lvtksqlite-6.1-gd \
        -lvtksys-6.1-gd \
        -lvtktiff-6.1-gd \
        -lvtkverdict-6.1-gd \
        -lvtkViewsContext2D-6.1-gd \
        -lvtkViewsCore-6.1-gd \
        -lvtkViewsGeovis-6.1-gd \
        -lvtkViewsInfovis-6.1-gd \
        -lvtkWrappingTools-6.1-gd \
        -lvtkzlib-6.1-gd

# OPENNI2
INCLUDEPATH += $$(DEPENDENCIES_INCLUDE)/OpenNI2

INCLUDEPATH += $$(DEPENDENCIES_LIB64)/OpenNI2
DEPENDPATH  += $$(DEPENDENCIES_LIB64)/OpenNI2

win32: LIBS += -L$$(DEPENDENCIES_LIB64)/OpenNI2 -lOpenNI2

# PCL
INCLUDEPATH += $$(DEPENDENCIES_LIB64)/pcl
DEPENDPATH  += $$(DEPENDENCIES_LIB64)/pcl

win32:CONFIG(release, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/pcl -lpcl_io_release \
    -lpcl_io_ply_release \
    -lpcl_filters_release \
    -lpcl_kdtree_release \
    -lpcl_registration_release \
    -lpcl_features_release \
    -lpcl_segmentation_release \
    -lpcl_common_release \
    -lpcl_visualization_release \
    -lpcl_search_release \
    -lpcl_keypoints_release \
    -lpcl_octree_release \
    -lpcl_outofcore_release \
    -lpcl_people_release \
    -lpcl_recognition_release \
    -lpcl_sample_consensus_release \
    -lpcl_surface_release \
    -lpcl_tracking_release

else:win32:CONFIG(debug, debug|release): LIBS += -L$$(DEPENDENCIES_LIB64)/pcl -lpcl_io_debug \
    -lpcl_io_ply_debug \
    -lpcl_filters_debug \
    -lpcl_kdtree_debug \
    -lpcl_registration_debug \
    -lpcl_features_debug \
    -lpcl_segmentation_debug \
    -lpcl_common_debug \
    -lpcl_visualization_debug \
    -lpcl_search_debug \
    -lpcl_keypoints_debug \
    -lpcl_octree_debug \
    -lpcl_outofcore_debug \
    -lpcl_people_debug \
    -lpcl_recognition_debug \
    -lpcl_sample_consensus_debug \
    -lpcl_surface_debug \
    -lpcl_tracking_debug
