include(avalon_controlTaskLib)
ADD_LIBRARY(${AVALON_CONTROL_TASKLIB_NAME} SHARED 
    ${AVALON_CONTROL_TASKLIB_SOURCES} pidcontroller.cpp )

add_dependencies(${AVALON_CONTROL_TASKLIB_NAME}
    regen-toolkit)


TARGET_LINK_LIBRARIES(${AVALON_CONTROL_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${AVALON_CONTROL_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${AVALON_CONTROL_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${AVALON_CONTROL_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${AVALON_CONTROL_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${AVALON_CONTROL_TASKLIB_HEADERS} pidcontroller.h
    DESTINATION include/orocos/avalon_control)

