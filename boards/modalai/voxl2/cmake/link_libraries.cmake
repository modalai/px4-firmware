
# Link against the public stub version of the proprietary fc sensor library
target_link_libraries(px4 PRIVATE
		dl
		px4_layer
		${module_libraries}
)
