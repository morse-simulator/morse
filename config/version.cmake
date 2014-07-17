# VERSION_COMPUTE
# ---------------
#
# Deduce automatically the version number.
# This mechanism makes sure that version number is always up-to-date and
# coherent (i.e. strictly increasing as commits are made).
#
# There is two cases:
# - the software comes from a release (stable version). In this case, the
#   software is retrieved through a tarball which does not contain the `.git'
#   directory. Hence, there is no way to search in the Git history to generate
#   the version number.
#   In this case, a 'version.py' file is put at the top-directory of the source
#   tree which contains the project version. 
#
# - the softwares comes from git (possibly unstable version).
#   'git describe' is used to retrieve the version number
#   (see 'man git-describe'). This tool generates a version number from the git
#   history. The version number follows this pattern:
#
#     TAG[-N-SHA1][-dirty]
#
#   TAG: last matching tag (i.e. last signed tag starting with v, i.e. v0.1)
#   N: number of commits since the last maching tag
#   SHA1: sha1 of the current commit
#   -dirty: added if the workig directory is dirty (there is some uncommitted
#           changes).
#
#   For stable releases, i.e. the current commit is a matching tag, -N-SHA1 is
#   omitted. If the HEAD is on the signed tag v0.1, the version number will be
#   0.1.
#
#   If the HEAD is two commits after v0.5 and the last commit is 034f6d...
#   The version number will be:
#   - 0.5-2-034f if there is no uncommitted changes,
#   - 0.5-2-034f-dirty if there is some uncommitted changes.
#
SET(PROJECT_STABLE False)

# Check if a version is embedded in the project.
IF(EXISTS ${SOURCE_DIR}/version.py)
	# Yes, use it. This is a stable version.
	FILE(COPY ${SOURCE_DIR}/version.py DESTINATION ${CMAKE_BINARY_DIR})
	SET(PROJECT_STABLE True)
	FILE(APPEND ${CMAKE_BINARY_DIR}/version.py "PROJECT_STABLE=${PROJECT_STABLE}\n")
ELSE(EXISTS ${SOURCE_DIR}/version.py)
	# No, there is no 'version.py' file. Deduce the version from git.

	# Search for git.
	FIND_PROGRAM(GIT git)
	IF(NOT GIT)
		MESSAGE("Warning: failed to compute the version number, git not found.")
		SET(PROJECT_VERSION UNKNOWN)
	ENDIF()

	# Run describe: search for *signed* tags starting with v, from the HEAD and
	# display only the first four characters of the commit id.
	EXECUTE_PROCESS(
		COMMAND ${GIT} describe --abbrev=4 HEAD
		WORKING_DIRECTORY ${SOURCE_DIR}
		RESULT_VARIABLE GIT_DESCRIBE_RESULT
		OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT
		ERROR_VARIABLE GIT_DESCRIBE_ERROR
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)

	# Run diff-index to check whether the tree is clean or not.
	EXECUTE_PROCESS(
		COMMAND ${GIT} diff-index --name-only HEAD
		WORKING_DIRECTORY ${SOURCE_DIR}
		RESULT_VARIABLE GIT_DIFF_INDEX_RESULT
		OUTPUT_VARIABLE GIT_DIFF_INDEX_OUTPUT
		ERROR_VARIABLE GIT_DIFF_INDEX_ERROR
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)

	EXECUTE_PROCESS(
		COMMAND ${GIT} rev-parse --abbrev-ref HEAD
		WORKING_DIRECTORY ${SOURCE_DIR}
		RESULT_VARIABLE GIT_BRANCH_RESULT
		OUTPUT_VARIABLE GIT_BRANCH_OUTPUT
		ERROR_VARIABLE GIT_BRANCH_ERROR
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)

	# Check if the tree is clean.
	IF(NOT GIT_DIFF_INDEX_RESULT AND NOT GIT_DIFF_INDEX_OUTPUT)
		SET(PROJECT_DIRTY False)
	ELSE()
		SET(PROJECT_DIRTY True)
	ENDIF()

	# Check if git describe worked and store the returned version number.
	IF(GIT_DESCRIBE_RESULT)
		MESSAGE(
			"Warning: failed to compute the version number,"
			" 'git describe' failed:\n"
			"\t" ${GIT_DESCRIBE_ERROR})
		SET(PROJECT_VERSION UNKNOWN)
	ELSE()
		# Get rid of the tag prefix to generate the final version.
		STRING(REGEX REPLACE "^v" "" PROJECT_VERSION "${GIT_DESCRIBE_OUTPUT}")
		IF(NOT PROJECT_VERSION)
			MESSAGE(
				"Warning: failed to compute the version number,"
				"'git describe' returned an empty string.")
			SET(PROJECT_VERSION UNKNOWN)
		ENDIF()

		# If there is a dash in the version number, it is an unstable release,
		# otherwise it is a stable release.
		# I.e. 1.0, 2, 0.1.3 are stable but 0.2.4-1-dg43 is unstable.
		STRING(REGEX MATCH "-" PROJECT_STABLE "${PROJECT_VERSION}")
		STRING(REGEX MATCH "_STABLE" BRANCH_STABLE "${GIT_BRANCH_OUTPUT}")
		IF(NOT PROJECT_STABLE STREQUAL "-" OR BRANCH_STABLE STREQUAL "_STABLE")
			SET(PROJECT_STABLE True)
		ELSE()
			SET(PROJECT_STABLE False)
		ENDIF()
	ENDIF()

	# Append dirty if the project is dirty.
	IF(PROJECT_DIRTY)
		SET(PROJECT_VERSION "${PROJECT_VERSION}-dirty")
	ENDIF()
	FILE(WRITE version.py "VERSION=\"${PROJECT_VERSION}\"\nPROJECT_STABLE=${PROJECT_STABLE}\n")
ENDIF(EXISTS ${SOURCE_DIR}/version.py)

