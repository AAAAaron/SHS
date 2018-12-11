add_library( SHS SHARED
  step.cpp
  ahrsangle.cpp
  ahrsrobust.cpp
  btwfilter.cpp
  ahrsqsmffilter.cpp
  ahrsqsmfbfilter.cpp
  phonerotateangle.cpp
  gyro.cpp
  pdrsim.cpp
  displayattitude.cpp
  config.cpp
  attitudeobserver.cpp
)

target_link_libraries( SHS
    ${THIRD_PARTY_LIBS}
)