set(SOURCES
  learning_machine.cpp
  learning_machine_blackbox.cpp
  learning_machine_factory.cpp
)
if (rosban_control_FOUND)
  set(SOURCES
    ${SOURCES}
    learning_machine_controller.cpp)
endif(rosban_control_FOUND)