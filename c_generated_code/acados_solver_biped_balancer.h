/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_biped_balancer_H_
#define ACADOS_SOLVER_biped_balancer_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define BIPED_BALANCER_NX     8
#define BIPED_BALANCER_NZ     0
#define BIPED_BALANCER_NU     3
#define BIPED_BALANCER_NP     0
#define BIPED_BALANCER_NP_GLOBAL     0
#define BIPED_BALANCER_NBX    0
#define BIPED_BALANCER_NBX0   8
#define BIPED_BALANCER_NBU    3
#define BIPED_BALANCER_NSBX   0
#define BIPED_BALANCER_NSBU   0
#define BIPED_BALANCER_NSH    0
#define BIPED_BALANCER_NSH0   0
#define BIPED_BALANCER_NSG    0
#define BIPED_BALANCER_NSPHI  0
#define BIPED_BALANCER_NSHN   0
#define BIPED_BALANCER_NSGN   0
#define BIPED_BALANCER_NSPHIN 0
#define BIPED_BALANCER_NSPHI0 0
#define BIPED_BALANCER_NSBXN  0
#define BIPED_BALANCER_NS     0
#define BIPED_BALANCER_NS0    0
#define BIPED_BALANCER_NSN    0
#define BIPED_BALANCER_NG     0
#define BIPED_BALANCER_NBXN   0
#define BIPED_BALANCER_NGN    0
#define BIPED_BALANCER_NY0    11
#define BIPED_BALANCER_NY     11
#define BIPED_BALANCER_NYN    8
#define BIPED_BALANCER_N      25
#define BIPED_BALANCER_NH     0
#define BIPED_BALANCER_NHN    0
#define BIPED_BALANCER_NH0    0
#define BIPED_BALANCER_NPHI0  0
#define BIPED_BALANCER_NPHI   0
#define BIPED_BALANCER_NPHIN  0
#define BIPED_BALANCER_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct biped_balancer_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost






    // constraints







} biped_balancer_solver_capsule;

ACADOS_SYMBOL_EXPORT biped_balancer_solver_capsule * biped_balancer_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_free_capsule(biped_balancer_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int biped_balancer_acados_create(biped_balancer_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int biped_balancer_acados_reset(biped_balancer_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of biped_balancer_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_create_with_discretization(biped_balancer_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_update_time_steps(biped_balancer_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_update_qp_solver_cond_N(biped_balancer_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_update_params(biped_balancer_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_update_params_sparse(biped_balancer_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_set_p_global_and_precompute_dependencies(biped_balancer_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int biped_balancer_acados_solve(biped_balancer_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT void biped_balancer_acados_batch_solve(biped_balancer_solver_capsule ** capsules, int N_batch);

ACADOS_SYMBOL_EXPORT void biped_balancer_acados_batch_set_flat(biped_balancer_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);
ACADOS_SYMBOL_EXPORT void biped_balancer_acados_batch_get_flat(biped_balancer_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);

ACADOS_SYMBOL_EXPORT void biped_balancer_acados_batch_eval_solution_sens_adj_p(biped_balancer_solver_capsule ** capsules, const char *field, int stage, double *out, int offset, int N_batch);
ACADOS_SYMBOL_EXPORT void biped_balancer_acados_batch_eval_params_jac(biped_balancer_solver_capsule ** capsules, int N_batch);


ACADOS_SYMBOL_EXPORT int biped_balancer_acados_free(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void biped_balancer_acados_print_stats(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int biped_balancer_acados_custom_update(biped_balancer_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *biped_balancer_acados_get_nlp_in(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *biped_balancer_acados_get_nlp_out(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *biped_balancer_acados_get_sens_out(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *biped_balancer_acados_get_nlp_solver(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *biped_balancer_acados_get_nlp_config(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *biped_balancer_acados_get_nlp_opts(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *biped_balancer_acados_get_nlp_dims(biped_balancer_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *biped_balancer_acados_get_nlp_plan(biped_balancer_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_biped_balancer_H_
