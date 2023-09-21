# Unit tests:
mola_add_test(
  TARGET  test-mola_erathos_slam-occgrid
  SOURCES test-occgrid.cpp
  LINK_LIBRARIES
  mola_erathos_slam
)
