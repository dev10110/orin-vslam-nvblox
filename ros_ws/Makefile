mode=RelWithDebInfo
.SILENT: build clean test default
.PHONY: build clean test default


# Default target when no argument is passed
default:
	@echo "Running default command: building all packages"
	colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=$(mode)

# clean all packages
clean:
	@echo "Cleaning env"
	rm -rf build install

# Test all packages
test:
	@echo "Testing packages"
	colcon test
	colcon test-result --verbose

# Clean specific package
clean-%:
	@echo "Cleaning package: $*"
	rm -rf build/$* install/$*

# Test specific package
test-%:
	@echo "Testing package: $*"
	colcon test --packages-select $*
	colcon test-result --verbose


# build a specific package
%:
	@echo "Building package: $*"
	colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=$(mode) --packages-select $*

#
#
# build:
# 	if [ -z "$(pkg)" ]; then \
#                 echo "building all"; \
#                 colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=$(mode); \
#         else \
#                 echo "building $(pkg)"; \
#                 colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=$(mode) --packages-select $(pkg);\
#         fi
#
# clean:
# 	if [ -z "$(pkg)" ]; then \
#                 echo "cleaning all"; \
#                 rm -r build install; \
#         else \
#                 echo "cleaning $(pkg)"; \
#                 rm -r build/$(pkg) install/$(pkg); \
#         fi
#
# test:  all
# 	if [ -z "$(pkg)" ]; then \
#                 echo "specify the pkg to test"; \
#         else \
#                 echo "testing $(pkg)"; \
#                 colcon test --packages-select $(pkg);\
#                 colcon test-result --verbose; \
#         fi
#
