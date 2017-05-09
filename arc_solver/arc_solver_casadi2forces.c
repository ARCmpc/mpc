/*  * CasADi to FORCES Template - missing information to be filled in by createCasadi.m 
 * (C) embotech GmbH, Zurich, Switzerland, 2013-16. All rights reserved.
 *
 * This file is part of the FORCES client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "arc_solver/include/arc_solver.h"    
    
/* prototyes for models */
extern void arc_solver_model_1(const arc_solver_FLOAT** arg, arc_solver_FLOAT** res);
extern void arc_solver_model_1_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row);
extern void arc_solver_model_9(const arc_solver_FLOAT** arg, arc_solver_FLOAT** res);
extern void arc_solver_model_9_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row);
    

/* copies data from sparse matrix into a dense one */
void sparse2fullCopy(int nrow, int ncol, const int* colidx, const int* row, arc_solver_FLOAT *data, arc_solver_FLOAT *Out)
{
    int i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++){
        for( j=colidx[i]; j < colidx[i+1]; j++ ){
            Out[i*nrow + row[j]] = data[j];
        }
    }
}

/* CasADi - FORCES interface */
void arc_solver_casadi2forces(arc_solver_FLOAT *x,        /* primal vars                                         */
                                 arc_solver_FLOAT *y,        /* eq. constraint multiplers                           */
                                 arc_solver_FLOAT *l,        /* ineq. constraint multipliers                        */
                                 arc_solver_FLOAT *p,        /* parameters                                          */
                                 arc_solver_FLOAT *f,        /* objective function (scalar)                         */
                                 arc_solver_FLOAT *nabla_f,  /* gradient of objective function                      */
                                 arc_solver_FLOAT *c,        /* dynamics                                            */
                                 arc_solver_FLOAT *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 arc_solver_FLOAT *h,        /* inequality constraints                              */
                                 arc_solver_FLOAT *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 arc_solver_FLOAT *H,        /* Hessian (column major)                              */
                                 int stage                      /* stage number (0 indexed)                            */
                  )
{
    /* CasADi input and output arrays */
    const arc_solver_FLOAT *in[4];
    arc_solver_FLOAT *out[7];
    
    /* temporary storage for casadi sparse output */
    arc_solver_FLOAT this_f;
    arc_solver_FLOAT nabla_f_sparse[2];
    
    
    arc_solver_FLOAT c_sparse[4];
    arc_solver_FLOAT nabla_c_sparse[16];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    int nrow, ncol;
    const int *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p; /* maybe should be made conditional */
    in[2] = l; /* maybe should be made conditional */     
    in[3] = y; /* maybe should be made conditional */
    
    /* set outputs for CasADi */
    out[0] = &this_f;
    out[1] = nabla_f_sparse;
                
	 if (stage >= 0 && stage < 8)
	 {
		 /* set inputs */
		 out[2] = c_sparse;
		 out[3] = nabla_c_sparse;
		 /* call CasADi */
		 arc_solver_model_1(in, out);

		 /* copy to dense */
		 if( nabla_f ){ arc_solver_model_1_sparsity(3, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f); }
		 if( c ){ arc_solver_model_1_sparsity(4, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, c_sparse, c); }
		 if( nabla_c ){ arc_solver_model_1_sparsity(5, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c); }
		 
	 }

	 if (stage >= 8 && stage < 9)
	 {
		 /* call CasADi */
		 arc_solver_model_9(in, out);

		 /* copy to dense */
		 if( nabla_f ){ arc_solver_model_9_sparsity(3, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f); }
		 
	 }

         
    
    /* add to objective */
    if( f ){
        *f += this_f;
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif