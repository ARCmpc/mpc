/*
ARC_solver : A fast customized optimization solver.

Copyright (C) 2013-2016 EMBOTECH GMBH [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#include <stdio.h>

#ifndef __ARC_solver_H__
#define __ARC_solver_H__

/* DATA TYPE ------------------------------------------------------------*/
typedef double ARC_solver_FLOAT;

typedef double ARC_solverINTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef ARC_solver_SET_PRINTLEVEL
#define ARC_solver_SET_PRINTLEVEL    (2)
#endif

/* timing */
#ifndef ARC_solver_SET_TIMING
#define ARC_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define ARC_solver_SET_MAXIT			(200)	

/* scaling factor of line search (FTB rule) */
#define ARC_solver_SET_FLS_SCALE		(ARC_solver_FLOAT)(0.99)      

/* maximum number of supported elements in the filter */
#define ARC_solver_MAX_FILTER_SIZE	(200) 

/* maximum number of supported elements in the filter */
#define ARC_solver_MAX_SOC_IT			(4) 

/* desired relative duality gap */
#define ARC_solver_SET_ACC_RDGAP		(ARC_solver_FLOAT)(0.0001)

/* desired maximum residual on equality constraints */
#define ARC_solver_SET_ACC_RESEQ		(ARC_solver_FLOAT)(1E-06)

/* desired maximum residual on inequality constraints */
#define ARC_solver_SET_ACC_RESINEQ	(ARC_solver_FLOAT)(1E-06)

/* desired maximum violation of complementarity */
#define ARC_solver_SET_ACC_KKTCOMPL	(ARC_solver_FLOAT)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define ARC_solver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define ARC_solver_MAXITREACHED (0)

/* NaN encountered in function evaluations */
#define ARC_solver_BADFUNCEVAL  (-6)

/* no progress in method possible */
#define ARC_solver_NOPROGRESS   (-7)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct ARC_solver_params
{
    /* vector of size 120 */
    ARC_solver_FLOAT x0[120];

    /* vector of size 4 */
    ARC_solver_FLOAT xinit[4];

    /* vector of size 60 */
    ARC_solver_FLOAT all_parameters[60];

} ARC_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct ARC_solver_output
{
    /* vector of size 6 */
    ARC_solver_FLOAT x01[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x02[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x03[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x04[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x05[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x06[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x07[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x08[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x09[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x10[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x11[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x12[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x13[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x14[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x15[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x16[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x17[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x18[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x19[6];

    /* vector of size 6 */
    ARC_solver_FLOAT x20[6];

} ARC_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct ARC_solver_info
{
    /* iteration number */
    int it;

	/* number of iterations needed to optimality (branch-and-bound) */
	int it2opt;
	
    /* inf-norm of equality constraint residuals */
    ARC_solver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    ARC_solver_FLOAT res_ineq;

    /* primal objective */
    ARC_solver_FLOAT pobj;	
	
    /* dual objective */
    ARC_solver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    ARC_solver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    ARC_solver_FLOAT rdgap;		

    /* duality measure */
    ARC_solver_FLOAT mu;

	/* duality measure (after affine step) */
    ARC_solver_FLOAT mu_aff;
	
    /* centering parameter */
    ARC_solver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    ARC_solver_FLOAT step_aff;
    
    /* step size (combined direction) */
    ARC_solver_FLOAT step_cc;    

	/* solvertime */
	ARC_solver_FLOAT solvetime;   

	/* time spent in function evaluations */
	ARC_solver_FLOAT fevalstime;  

} ARC_solver_info;








/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*ARC_solver_ExtFunc)(ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, ARC_solver_FLOAT*, int);

int ARC_solver_solve(ARC_solver_params* params, ARC_solver_output* output, ARC_solver_info* info, FILE* fs, ARC_solver_ExtFunc ARC_solver_evalExtFunctions);	


#ifdef __cplusplus
}
#endif

#endif