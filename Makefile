#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := door_sensor

add_compile_options(-fshort-enums)

include $(IDF_PATH)/make/project.mk
