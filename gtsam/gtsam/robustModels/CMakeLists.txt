# Install headers
file(GLOB gnssNavigation_headers "*.h")
install(FILES ${robustModels_headers} DESTINATION include/gtsam/robustModels)

# Add all tests
add_subdirectory(tests)
