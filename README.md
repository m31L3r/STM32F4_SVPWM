# STM32F4_FOC
Field oriented control with STM32F4 Discovery Board


/*=============================================================================

 |   Assignment:  TPA_Frequenzumrichter

 |

 |	Created on:	10.01.2016

 |          Author:	Michael Meiler

 |    	  Language:	C, Gnu Arm Toolchain C-Compiler

 |

 |	To Compile:	Make sure libc and libm are defined for Linker, as well as arm_cortexM4_math lib

 |   Compiler Flag:	-mfloat-abi=hard for FPU and in settings->targetprocessor set Float ABI = hard

 |

 |   	Instructor:  	NAME OF YOUR COURSE'S INSTRUCTOR

 |   	  Due Date:  	DATE AND TIME THAT THIS PROGRAM IS/WAS DUE TO BE SUBMITTED

 |

 +-----------------------------------------------------------------------------

 |

 |  Description:

 |

 |        Input:  DESCRIBE THE INPUT THAT THE PROGRAM REQUIRES.

 |

 |       Output:  DESCRIBE THE OUTPUT THAT THE PROGRAM PRODUCES.

 |

 |    Algorithm:  OUTLINE THE APPROACH USED BY THE PROGRAM TO SOLVE THE PROBLEM.

 |

 |   Required Features Not Included:  DESCRIBE HERE ANY REQUIREMENTS OF 

 |			THE ASSIGNMENT THAT THE PROGRAM DOES NOT ATTEMPT TO SOLVE.

 |

 |   Known Bugs:  IF THE PROGRAM DOES NOT FUNCTION CORRECTLY IN SOME

 |      	  SITUATIONS, DESCRIBE THE SITUATIONS AND PROBLEMS HERE.

 |

 *===========================================================================*/





/*-----------------------------------Space Vector Modulation -----

|  Function:	svpwm()

|

|  Purpose:	This subroutine determines the sector (1 out of 6 possible)

|		which the voltage vector should be modulated,

|		then calculates the on/off times, the counter values for switching respectively.

|		Addresses of calculated values are saved in pointer array "switchtime".

|

|

|

|  Description:	_______________________________________________________

|			   |         __|	     __|	     __|

|			   |	   |/  |	   |/  | 	   |/  |

|			   |    0__|  /_\       1__|  /_\       2__|  /_\

|			   |	   |\__| 	   |\__| 	   |\__|

|	 		 * | *	       |	       |	       |

|     		      *    |    *      +---------------|---------------|-------u

|                    *     |     *     |	       +---------------|-------v

|                    *     |     *     |	       |	       +-------w

|                     *    |    *      |	       |	       |

|                        * | *	     __|	     __|	     __|

|			   |  	   |/  |	   |/  |	   |/  |

|			   |    3__|  /_\       4__|  /_\       5__|  /_\

|			   |	   |\__|	   |\__|	   |\__|

|			   |___________|_______________|_______________|

|

|

|  Parameters:	[in]	None.

|			Uses global variables (Theta, U_alpha, U_beta), without changing them.

|

|		[out]	uint16_t *switchtime[3]

|

|  Returns:  	No return function used, but updates counter values in pointer array "switchtime".

|

*-------------------------------------------------------------------*/
