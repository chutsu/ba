#ifndef UTIL_H
#define UTIL_H

#define PRECISION 1
#define MAX_LINE_LENGTH 9046
#define USE_CBLAS
#define USE_LAPACK

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include <sys/time.h>

#ifdef USE_CBLAS
#include <cblas.h>
#endif

#ifdef USE_LAPACK
#include <lapacke.h>
#endif

/******************************************************************************
 *                                LOGGING
 ******************************************************************************/

/* DEBUG */
#ifdef NDEBUG
#define DEBUG(M, ...)
#else
#define DEBUG(M, ...)                                                          \
  fprintf(stderr, "[DEBUG] %s:%d: " M "\n", __func__, __LINE__, ##__VA_ARGS__)
#endif

/* LOG */
#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr, "[ERROR] [%s] " M "\n", __func__, ##__VA_ARGS__)
#define LOG_WARN(M, ...) fprintf(stderr, "[WARN] " M "\n", ##__VA_ARGS__)
#define LOG_INFO(M, ...) fprintf(stderr, "[INFO] " M "\n", ##__VA_ARGS__)

/* FATAL */
#define FATAL(M, ...)                                                          \
  fprintf(stderr, "[FATAL] " M "\n", ##__VA_ARGS__);                           \
  exit(-1);

/* CHECK */
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    log_err(M, ##__VA_ARGS__);                                                 \
    goto error;                                                                \
  }

/******************************************************************************
 *                                   DATA
 ******************************************************************************/

#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Precision not defined!"
#endif

char *malloc_string(const char *s);
int dsv_rows(const char *fp);
int dsv_cols(const char *fp, const char delim);
char **dsv_fields(const char *fp, const char delim, int *nb_fields);
real_t **dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols);
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols);
int **load_iarrays(const char *csv_path, int *nb_arrays);
real_t **load_darrays(const char *csv_path, int *nb_arrays);
real_t *load_vector(const char *file_path);

/******************************************************************************
 *                                 MATHS
 ******************************************************************************/

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

float randf(float a, float b);
real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);
int fltcmp(const real_t x, const real_t y);
real_t pythag(const real_t a, const real_t b);
real_t lerp(const real_t a, const real_t b, const real_t t);
void lerp3(const real_t *a, const real_t *b, const real_t t, real_t *x);
real_t sinc(const real_t x);

/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix,
                  const real_t *data,
                  const size_t m,
                  const size_t n);
void print_vector(const char *prefix, const real_t *data, const size_t length);

void eye(real_t *A, const size_t m, const size_t n);
void ones(real_t *A, const size_t m, const size_t n);
void zeros(real_t *A, const size_t m, const size_t n);

real_t *mat_new(const size_t m, const size_t n);
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n);
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol);
int mat_save(const char *save_path, const real_t *A, const int m, const int n);
real_t *mat_load(const char *save_path, int *nb_rows, int *nb_cols);
void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val);
real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j);
void mat_copy(const real_t *src, const int m, const int n, real_t *dest);
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block);
void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const real_t *block);
void mat_diag_get(const real_t *A, const int m, const int n, real_t *d);
void mat_diag_set(real_t *A, const int m, const int n, const real_t *d);
void mat_triu(const real_t *A, const size_t n, real_t *U);
void mat_tril(const real_t *A, const size_t n, real_t *L);
real_t mat_trace(const real_t *A, const size_t m, const size_t n);
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t);
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale);

real_t *vec_new(const size_t length);
void vec_copy(const real_t *src, const size_t length, real_t *dest);
int vec_equals(const real_t *x, const real_t *y, const size_t length);
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t length);
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t length);
void vec_scale(real_t *x, const size_t length, const real_t scale);
real_t vec_norm(const real_t *x, const size_t length);

void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C);
void skew(const real_t x[3], real_t A[3 * 3]);
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n);
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n);
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int print);

#ifdef USE_CBLAS
void cblas_dot(const real_t *A,
               const size_t A_m,
               const size_t A_n,
               const real_t *B,
               const size_t B_m,
               const size_t B_n,
               real_t *C);
#endif

/******************************************************************************
 *                                   SVD
 ******************************************************************************/

int svd(real_t *A, int m, int n, real_t *U, real_t *s, real_t *V_t);
int svdcomp(real_t *A, int m, int n, real_t *w, real_t *V);
int pinv(real_t *A, const int m, const int n, real_t *A_inv);

#ifdef USE_LAPACK

#endif

/******************************************************************************
 *                                   CHOL
 ******************************************************************************/

void chol(const real_t *A, const size_t n, real_t *L);
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n);

#ifdef USE_LAPACK
void lapack_chol_solve(const real_t *A,
                       const real_t *b,
                       real_t *x,
                       const size_t n);
#endif

/******************************************************************************
 *                                   TIME
 ******************************************************************************/

typedef uint64_t timestamp_t;

struct timespec tic();
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
float time_now();

/******************************************************************************
 *                                TRANSFORMS
 ******************************************************************************/

void tf(const real_t params[7], real_t T[4 * 4]);
void tf_params(const real_t T[4 * 4], real_t params[7]);
void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]);
void tf_trans_set(real_t T[4 * 4], const real_t r[3]);
void tf_trans_get(const real_t T[4 * 4], real_t r[3]);
void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]);
void tf_quat_get(const real_t T[4 * 4], real_t q[4]);
void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]);
void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]);
void tf_hpoint(const real_t T[4 * 4], const real_t p[4], real_t retval[4]);
void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i);
void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i);
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R);
void euler321(const real_t euler[3], real_t C[3 * 3]);
void rot2quat(const real_t C[3 * 3], real_t q[4]);
void quat2euler(const real_t q[4], real_t euler[3]);
void quat2rot(const real_t q[4], real_t C[3 * 3]);
void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_delta(const real_t dalpha[3], real_t dq[4]);

/******************************************************************************
 *                                 IMAGE
 ******************************************************************************/

typedef struct image_t {
  int width;
  int height;
  uint8_t *data;
} image_t;

void image_init(image_t *img, uint8_t *data, int width, int height);

/******************************************************************************
 *                                  CV
 ******************************************************************************/

/******************************** RADTAN **************************************/

void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]);
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]);

/********************************* EQUI ***************************************/

void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]);
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]);

/******************************** PINHOLE *************************************/

real_t pinhole_focal(const int image_width, const real_t fov);
int pinhole_project(const real_t params[4], const real_t p_C[3], real_t x[2]);

void pinhole_point_jacobian(const real_t params[4], real_t J_point[2 * 3]);
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]);

/**************************** PINHOLE-RADTAN4 *********************************/

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]);
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]);
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]);

/***************************** PINHOLE-EQUI4 **********************************/

void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]);
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]);
void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]);

#endif // ZERO_H
