# determinism_test.cmake — run the sim binary twice, fail on any diff.
# Usage: cmake -DSIM_BIN=<path> -P determinism_test.cmake
# The sim has no wall clock and fixed VFS assets, so two runs must be
# byte-identical — this is the Phase-1 done-criteria check and the
# foundation for shareable replays later (identical inputs => identical run).

if(NOT SIM_BIN)
  message(FATAL_ERROR "determinism_test.cmake needs -DSIM_BIN")
endif()

execute_process(COMMAND ${SIM_BIN} OUTPUT_VARIABLE RUN1 RESULT_VARIABLE RC1)
execute_process(COMMAND ${SIM_BIN} OUTPUT_VARIABLE RUN2 RESULT_VARIABLE RC2)

if(NOT RC1 EQUAL 0 OR NOT RC2 EQUAL 0)
  message(FATAL_ERROR "sim binary failed (exit ${RC1} / ${RC2})")
endif()

if(NOT RUN1 STREQUAL RUN2)
  message(FATAL_ERROR "sim runs are NOT deterministic — outputs differ")
endif()

message(STATUS "determinism OK: two runs byte-identical")
