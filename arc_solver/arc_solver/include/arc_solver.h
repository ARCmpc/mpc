/*
arc_solver : A fast customized optimization solver.

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

#ifndef __arc_solver_H__
#define __arc_solver_H__

/* DATA TYPE ------------------------------------------------------------*/
typedef double arc_solver_FLOAT;

typedef double arc_solverINTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef arc_solver_SET_PRINTLEVEL
#define arc_solver_SET_PRINTLEVEL    (1)
#endif

/* timing */
#ifndef arc_solver_SET_TIMING
#define arc_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define arc_solver_SET_MAXIT			(1000)	

/* scaling factor of line search (FTB rule) */
#define arc_solver_SET_FLS_SCALE		(arc_solver_FLOAT)(0.99)      

/* maximum number of supported elements in the filter */
#define arc_solver_MAX_FILTER_SIZE	(1000) 

/* maximum number of supported elements in the filter */
#define arc_solver_MAX_SOC_IT			(4) 

/* desired relative duality gap */
#define arc_solver_SET_ACC_RDGAP		(arc_solver_FLOAT)(0.0001)

/* desired maximum residual on equality constraints */
#define arc_solver_SET_ACC_RESEQ		(arc_solver_FLOAT)(1E-06)

/* desired maximum residual on inequality constraints */
#define arc_solver_SET_ACC_RESINEQ	(arc_solver_FLOAT)(1E-06)

/* desired maximum violation of complementarity */
#define arc_solver_SET_ACC_KKTCOMPL	(arc_solver_FLOAT)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define arc_solver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define arc_solver_MAXITREACHED (0)

/* NaN encountered in function evaluations */
#define arc_solver_BADFUNCEVAL  (-6)

/* no progress in method possible */
#define arc_solver_NOPROGRESS   (-7)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct arc_solver_params
{
    /* vector of size 160 */
    arc_solver_FLOAT x0[160];

    /* vector of size 6 */
    arc_solver_FLOAT xinit[6];

    /* vector of size 220 */
    arc_solver_FLOAT all_parameters[220];

} arc_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct arc_solver_output
{
    /* vector of size 8 */
    arc_solver_FLOAT x01[8];

    /* vector of size 8 */
    arc_solver_FLOAT x02[8];

    /* vector of size 8 */
    arc_solver_FLOAT x03[8];

    /* vector of size 8 */
    arc_solver_FLOAT x04[8];

    /* vector of size 8 */
    arc_solver_FLOAT x05[8];

    /* vector of size 8 */
    arc_solver_FLOAT x06[8];

    /* vector of size 8 */
    arc_solver_FLOAT x07[8];

    /* vector of size 8 */
    arc_solver_FLOAT x08[8];

    /* vector of size 8 */
    arc_solver_FLOAT x09[8];

    /* vector of size 8 */
    arc_solver_FLOAT x10[8];

    /* vector of size 8 */
    arc_solver_FLOAT x11[8];

    /* vector of size 8 */
    arc_solver_FLOAT x12[8];

    /* vector of size 8 */
    arc_solver_FLOAT x13[8];

    /* vector of size 8 */
    arc_solver_FLOAT x14[8];

    /* vector of size 8 */
    arc_solver_FLOAT x15[8];

    /* vector of size 8 */
    arc_solver_FLOAT x16[8];

    /* vector of size 8 */
    arc_solver_FLOAT x17[8];

    /* vector of size 8 */
    arc_solver_FLOAT x18[8];

    /* vector of size 8 */
    arc_solver_FLOAT x19[8];

    /* vector of size 8 */
    arc_solver_FLOAT x20[8];

} arc_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct arc_solver_info
{
    /* iteration number */
    int it;

	/* number of iterations needed to optimality (branch-and-bound) */
	int it2opt;
	
    /* inf-norm of equality constraint residuals */
    arc_solver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    arc_solver_FLOAT res_ineq;

    /* primal objective */
    arc_solver_FLOAT pobj;	
	
    /* dual objective */
    arc_solver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    arc_solver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    arc_solver_FLOAT rdgap;		

    /* duality measure */
    arc_solver_FLOAT mu;

	/* duality measure (after affine step) */
    arc_solver_FLOAT mu_aff;
	
    /* centering parameter */
    arc_solver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    arc_solver_FLOAT step_aff;
    
    /* step size (combined direction) */
    arc_solver_FLOAT step_cc;    

	/* solvertime */
	arc_solver_FLOAT solvetime;   

	/* time spent in function evaluations */
	arc_solver_FLOAT fevalstime;  

} arc_solver_info;








/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*arc_solver_ExtFunc)(arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, arc_solver_FLOAT*, int);

int arc_solver_solve(arc_solver_params* params, arc_solver_output* output, arc_solver_info* info, FILE* fs, arc_solver_ExtFunc arc_solver_evalExtFunctions);	


#ifdef __cplusplus
}
#endif

#endif