/*
 *   ARDrone_IMU_Acc_Y.cpp Simple C-MEX S-function for function call.
 *
 *   ABSTRACT:
 *     The purpose of this sfunction is to call a simple legacy
 *     function during simulation:
 *
 *        int16 y1 = IMU_Get_Acceleration_Y()
 *
 *   Simulink version           : 8.1 (R2013a) 13-Feb-2013
 *   C++ source code generated on : 18-Dec-2013 14:36:27

 * THIS S-FUNCTION IS GENERATED BY THE LEGACY CODE TOOL AND MAY NOT WORK IF MODIFIED

 */

/*
   %%%-MATLAB_Construction_Commands_Start
   def = legacy_code('initialize');
   def.SFunctionName = 'ARDrone_IMU_Acc_Y';
   def.OutputFcnSpec = 'int16 y1 = IMU_Get_Acceleration_Y()';
   def.StartFcnSpec = 'IMU_Initialization()';
   def.HeaderFiles = {'imu.h'};
   def.SourceFiles = {'imu.cpp'};
   def.IncPaths = {'.'};
   def.Options.language = 'C++';
   legacy_code('sfcn_cmex_generate', def);
   legacy_code('compile', def);
   %%%-MATLAB_Construction_Commands_End
 */

/*
 * Must specify the S_FUNCTION_NAME as the name of the S-function.
 */
#define S_FUNCTION_NAME                ARDrone_IMU_Acc_Y
#define S_FUNCTION_LEVEL               2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/* Ensure that this S-Function is compiled with a C++ compiler */
#ifndef __cplusplus
#error This S-Function must be compiled with a C++ compiler. Enter mex('-setup') in the MATLAB Command Window to configure a C++ compiler.
#endif

/*
 * Specific header file(s) required by the legacy code function.
 */
#include "imu.h"

/* Function: mdlInitializeSizes ===========================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
  /* Number of expected parameters */
  ssSetNumSFcnParams(S, 0);

  /*
   * Set the number of pworks.
   */
  ssSetNumPWork(S, 0);

  /*
   * Set the number of dworks.
   */
  if (!ssSetNumDWork(S, 0))
    return;

  /*
   * Set the number of input ports.
   */
  if (!ssSetNumInputPorts(S, 0))
    return;

  /*
   * Set the number of output ports.
   */
  if (!ssSetNumOutputPorts(S, 1))
    return;

  /*
   * Configure the output port 1
   */
  ssSetOutputPortDataType(S, 0, SS_INT16);
  ssSetOutputPortWidth(S, 0, 1);
  ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
  ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
  ssSetOutputPortOutputExprInRTW(S, 0, 1);

  /*
   * Register reserved identifiers to avoid name conflict
   */
  if (ssRTWGenIsCodeGen(S) || ssGetSimMode(S)==SS_SIMMODE_EXTERNAL) {
    /*
     * Register reserved identifier for StartFcnSpec
     */
    ssRegMdlInfo(S, "IMU_Initialization", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath
                 (S));

    /*
     * Register reserved identifier for OutputFcnSpec
     */
    ssRegMdlInfo(S, "IMU_Get_Acceleration_Y", MDL_INFO_ID_RESERVED, 0, 0,
                 ssGetPath(S));

    /*
     * Register reserved identifier for wrappers
     */
    if (ssRTWGenIsModelReferenceSimTarget(S)) {
      /*
       * Register reserved identifier for StartFcnSpec for SimulationTarget
       */
      ssRegMdlInfo(S, "ARDrone_IMU_Acc_Y_wrapper_start", MDL_INFO_ID_RESERVED, 0,
                   0, ssGetPath(S));

      /*
       * Register reserved identifier for OutputFcnSpec for SimulationTarget
       */
      ssRegMdlInfo(S, "ARDrone_IMU_Acc_Y_wrapper_output", MDL_INFO_ID_RESERVED,
                   0, 0, ssGetPath(S));
    }
  }

  /*
   * This S-function can be used in referenced model simulating in normal mode.
   */
  ssSetModelReferenceNormalModeSupport(S, MDL_START_AND_MDL_PROCESS_PARAMS_OK);

  /*
   * Set the number of sample time.
   */
  ssSetNumSampleTimes(S, 1);

  /*
   * All options have the form SS_OPTION_<name> and are documented in
   * matlabroot/simulink/include/simstruc.h. The options should be
   * bitwise or'd together as in
   *   ssSetOptions(S, (SS_OPTION_name1 | SS_OPTION_name2))
   */
  ssSetOptions(S,
               SS_OPTION_USE_TLC_WITH_ACCELERATOR |
               SS_OPTION_CAN_BE_CALLED_CONDITIONALLY |
               SS_OPTION_EXCEPTION_FREE_CODE |
               SS_OPTION_WORKS_WITH_CODE_REUSE |
               SS_OPTION_SFUNCTION_INLINED_FOR_RTW |
               SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);
}

/* Function: mdlInitializeSampleTimes =====================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);

#if defined(ssSetModelReferenceSampleTimeDefaultInheritance)

  ssSetModelReferenceSampleTimeDefaultInheritance(S);

#endif

}

#define MDL_START
#if defined(MDL_START)

/* Function: mdlStart =====================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
  /*
   * Call the legacy code function
   */
  IMU_Initialization();
}

#endif

/* Function: mdlOutputs ===================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector(s),
 *    ssGetOutputPortSignal.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  /*
   * Get access to Parameter/Input/Output/DWork/size information
   */
  int16_T *y1 = (int16_T *) ssGetOutputPortSignal(S, 0);

  /*
   * Call the legacy code function
   */
  *y1 = IMU_Get_Acceleration_Y();
}

/* Function: mdlTerminate =================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.
 */
static void mdlTerminate(SimStruct *S)
{
}

/*
 * Required S-function trailer
 */
#ifdef MATLAB_MEX_FILE
# include "simulink.c"
#else
# include "cg_sfun.h"
#endif
