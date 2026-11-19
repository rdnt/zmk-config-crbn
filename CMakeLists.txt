if(CONFIG_ZMK_KEYGRID_SCREEN)
  target_sources(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/config/keygrid_status_screen.c)
endif()