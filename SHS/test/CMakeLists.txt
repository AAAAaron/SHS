add_executable( pdrsim main.cpp )
target_link_libraries( pdrsim SHS )

add_executable( disAtt dis_att.cpp )
target_link_libraries( disAtt SHS )

add_executable( testp test_attitude.cpp )
target_link_libraries( testp SHS )