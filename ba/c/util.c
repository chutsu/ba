#include "util.h"

/******************************************************************************
 *                                   DATA
 ******************************************************************************/

char *malloc_string(const char *s) {
  char *retval = malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

int dsv_rows(const char *fp) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  /* Loop through lines */
  int nb_rows = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return nb_rows;
}

int dsv_cols(const char *fp, const char delim) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  /* Get line that isn't the header */
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  /* Parse line to obtain number of elements */
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < MAX_LINE_LENGTH; i++) {
    if (line[i] == delim) {
      found_separator = 1;
      nb_elements++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

char **dsv_fields(const char *fp, const char delim, int *nb_fields) {
  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Get last header line */
  char field_line[MAX_LINE_LENGTH] = {0};
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      strcpy(field_line, line);
    }
  }

  /* Parse fields */
  *nb_fields = dsv_cols(fp, delim);
  char **fields = malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    /* Ignore # and ' ' */
    if (c == '#' || c == delim) {
      continue;
    }

    if (c == ',' || c == '\n') {
      /* Add field name to fields */
      fields[field_idx] = malloc_string(field_name);
      memset(field_name, '\0', sizeof(char) * 100);
      field_idx++;
    } else {
      /* Append field name */
      field_name[strlen(field_name)] = c;
    }
  }

  /* Cleanup */
  fclose(infile);

  return fields;
}

real_t **dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);

  /* Obtain number of rows and columns in dsv data */
  *nb_rows = dsv_rows(fp);
  *nb_cols = dsv_cols(fp, delim);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for dsv data */
  real_t **data = malloc(sizeof(real_t *) * *nb_rows);
  for (int i = 0; i < *nb_rows; i++) {
    data[i] = malloc(sizeof(real_t) * *nb_cols);
  }

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[row_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return data;
}

real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  return dsv_data(fp, ',', nb_rows, nb_cols);
}

static int *parse_iarray_line(char *line) {
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  int *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size + 1, sizeof(int));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

int **load_iarrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  int **array = calloc(*nb_arrays, sizeof(int *));

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_iarray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

static real_t *parse_darray_line(char *line) {
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  real_t *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size, sizeof(real_t));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

real_t **load_darrays(const char *csv_path, int *nb_arrays) {
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  real_t **array = calloc(*nb_arrays, sizeof(real_t *));

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_darray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

real_t *load_vector(const char *file_path) {
  /* Open file */
  FILE *csv = fopen(file_path, "r");
  if (csv == NULL) {
    return NULL;
  }

  /* Get number of lines in file */
  size_t nb_lines = 0;
  char buf[MAX_LINE_LENGTH] = {0};
  while (fgets(buf, MAX_LINE_LENGTH, csv)) {
    nb_lines++;
  }
  rewind(csv);

  /* Load vector */
  real_t *v = malloc(sizeof(real_t) * nb_lines);
  for (size_t i = 0; i < nb_lines; i++) {
#if PRECISION == 1
    int retval = fscanf(csv, "%f", &v[i]);
#elif PRECISION == 2
    int retval = fscanf(csv, "%le", &v[i]);
#endif
    if (retval != 1) {
      return NULL;
    }
  }
  fclose(csv);

  return v;
}

/******************************************************************************
 *                                 MATHS
 ******************************************************************************/

float randf(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

int fltcmp(const real_t x, const real_t y) {
  if (fabs(x - y) < 1e-6) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

real_t pythag(const real_t a, const real_t b) {
  real_t at = fabs(a);
  real_t bt = fabs(b);
  real_t ct = 0.0;
  real_t result = 0.0;

  if (at > bt) {
    ct = bt / at;
    result = at * sqrt(1.0 + ct * ct);
  } else if (bt > 0.0) {
    ct = at / bt;
    result = bt * sqrt(1.0 + ct * ct);
  } else {
    result = 0.0;
  }

  return result;
}

real_t lerp(const real_t a, const real_t b, const real_t t) {
  return a * (1.0 - t) + b * t;
}

void lerp3(const real_t *a, const real_t *b, const real_t t, real_t *x) {
  x[0] = lerp(a[0], b[0], t);
  x[1] = lerp(a[1], b[1], t);
  x[2] = lerp(a[2], b[2], t);
}

real_t sinc(const real_t x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    const real_t c2 = 1.0 / 6.0;
    const real_t c4 = 1.0 / 120.0;
    const real_t c6 = 1.0 / 5040.0;
    const real_t x2 = x * x;
    const real_t x4 = x2 * x2;
    const real_t x6 = x2 * x2 * x2;
    return 1.0 - c2 * x2 + c4 * x4 - c6 * x6;
  }
}

/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix,
                  const real_t *data,
                  const size_t m,
                  const size_t n) {
  assert(prefix != NULL);
  assert(data != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0;
  printf("%s:\n", prefix);
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      printf("%.4f\t", data[idx]);
      idx++;
    }
    printf("\n");
  }
}

void print_vector(const char *prefix, const real_t *data, const size_t length) {
  assert(prefix != NULL);
  assert(data != NULL);
  assert(length != 0);

  size_t idx = 0;
  printf("%s: ", prefix);
  for (size_t i = 0; i < length; i++) {
    printf("%.4f\t", data[idx]);
    idx++;
  }
  printf("\n");
}

void eye(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = (i == j) ? 1.0 : 0.0;
      idx++;
    }
  }
}

void ones(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 1.0;
      idx++;
    }
  }
}

void zeros(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 0.0;
      idx++;
    }
  }
}

real_t *mat_new(const size_t m, const size_t n) {
  return calloc(m * n, sizeof(real_t));
}

int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n) {
  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      int retval = fltcmp(A[index], B[index]);
      if (retval != 0) {
        printf("Failed at index[%zu]\n", index);
        return retval;
      }
      index++;
    }
  }

  return 0;
}

int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol) {
  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(A[index] - B[index]) > tol) {
        printf("Failed at index[%zu]\n", index);
        return -1;
      }
      index++;
    }
  }

  return 0;
}

int mat_save(const char *save_path, const real_t *A, const int m, const int n) {
  FILE *csv_file = fopen(save_path, "w");
  if (csv_file == NULL) {
    return -1;
  }

  int idx = 0;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fprintf(csv_file, "%e", A[idx]);
      idx++;
      if ((j + 1) != n) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
  }
  fclose(csv_file);

  return 0;
}

real_t *mat_load(const char *mat_path, int *nb_rows, int *nb_cols) {
  /* Obtain number of rows and columns in csv data */
  *nb_rows = dsv_rows(mat_path);
  *nb_cols = dsv_cols(mat_path, ',');
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for csv data */
  real_t *A = malloc(sizeof(real_t) * *nb_rows * *nb_cols);

  /* Load file */
  FILE *infile = fopen(mat_path, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;
  int idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        A[idx] = strtod(entry, NULL);
        idx++;

        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return A;
}

void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val) {
  assert(A != NULL);
  assert(stride != 0);
  A[(i * stride) + j] = val;
}

real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j) {
  assert(A != NULL);
  assert(stride != 0);
  return A[(i * stride) + j];
}

void mat_copy(const real_t *src, const int m, const int n, real_t *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block) {
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      block[idx] = mat_val(A, stride, i, j);
      idx++;
    }
  }
}

void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const real_t *block) {
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      mat_set(A, stride, i, j, block[idx]);
      idx++;
    }
  }
}

void mat_diag_get(const real_t *A, const int m, const int n, real_t *d) {
  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        d[vec_index] = A[mat_index];
        vec_index++;
      }
      mat_index++;
    }
  }
}

void mat_diag_set(real_t *A, const int m, const int n, const real_t *d) {
  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        A[mat_index] = d[vec_index];
        vec_index++;
      } else {
        A[mat_index] = 0.0;
      }
      mat_index++;
    }
  }
}

void mat_triu(const real_t *A, const size_t n, real_t *U) {
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      U[i * n + j] = (j >= i) ? A[i * n + j] : 0.0;
    }
  }
}

void mat_tril(const real_t *A, const size_t n, real_t *L) {
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      L[i * n + j] = (j <= i) ? A[i * n + j] : 0.0;
    }
  }
}

real_t mat_trace(const real_t *A, const size_t m, const size_t n) {
  real_t tr = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      tr += (i == j) ? A[i * n + j] : 0.0;
    }
  }
  return tr;
}

void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A_t, m, j, i, mat_val(A, n, i, j));
    }
  }
}

void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(C, n, i, j, mat_val(A, n, i, j) + mat_val(B, n, i, j));
    }
  }
}

void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(C, n, i, j, mat_val(A, n, i, j) - mat_val(B, n, i, j));
    }
  }
}

void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale) {
  assert(A != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A, n, i, j, mat_val(A, n, i, j) * scale);
    }
  }
}

real_t *vec_new(const size_t length) { return calloc(length, sizeof(real_t)); }

void vec_copy(const real_t *src, const size_t length, real_t *dest) {
  for (size_t i = 0; i < length; i++) {
    dest[i] = src[i];
  }
}

int vec_equals(const real_t *x, const real_t *y, const size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (fltcmp(x[i], y[i]) != 0) {
      return -1;
    }
  }
  return 0;
}

void vec_add(const real_t *x, const real_t *y, real_t *z, size_t length) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(length > 0);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] + y[i];
  }
}

void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t length) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(length > 0);

  for (size_t i = 0; i < length; i++) {
    z[i] = x[i] - y[i];
  }
}

void vec_scale(real_t *x, const size_t length, const real_t scale) {
  for (size_t i = 0; i < length; i++) {
    x[i] = x[i] * scale;
  }
}

real_t vec_norm(const real_t *x, const size_t length) {
  real_t sum = 0.0;
  for (size_t i = 0; i < length; i++) {
    sum += x[i] * x[i];
  }
  return sqrt(sum);
}

void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C) {
  assert(A != NULL && B != NULL && A != C && B != C);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }
}

void skew(const real_t x[3], real_t A[3 * 3]) {
  /* First row */
  A[0] = 0.0;
  A[1] = -x[2];
  A[2] = x[1];

  /* Second row */
  A[3] = x[2];
  A[4] = 0.0;
  A[5] = -x[0];

  /* Third row */
  A[6] = -x[1];
  A[7] = x[0];
  A[8] = 0.0;
}

void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n) {
  for (size_t i = 0; i < n; i++) {
    real_t alpha = b[i];
    for (size_t j = 0; j < i; j++) {
      alpha -= L[i * n + j] * y[j];
    }
    y[i] = alpha / L[i * n + i];
  }
}

void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n) {
  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];
    for (int j = i; j < (int) n; j++) {
      alpha -= U[i * n + j] * x[j];
    }
    x[i] = alpha / U[i * n + i];
  }
}

int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int print) {
  int retval = 0;
  int ok = 1;
  real_t *delta = mat_new(m, n);
  mat_sub(fdiff, jac, delta, m, n);

  /* Check if any of the values are beyond the tol */
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(mat_val(delta, n, i, j)) >= tol) {
        ok = 0;
      }
    }
  }

  /* Print result */
  if (ok == 0) {
    if (print) {
      LOG_ERROR("Bad jacobian [%s]!\n", jac_name);
      print_matrix("analytical jac", jac, m, n);
      printf("\n");
      print_matrix("num diff jac", fdiff, m, n);
      printf("\n");
      print_matrix("difference matrix", delta, m, n);
      printf("\n");
    }
    retval = -1;
  } else {
    if (print) {
      printf("Check [%s] ok!\n", jac_name);
    }
    retval = 0;
  }

  return retval;
}

#ifdef USE_CBLAS
void cblas_dot(const real_t *A,
               const size_t A_m,
               const size_t A_n,
               const real_t *B,
               const size_t B_m,
               const size_t B_n,
               real_t *C) {
  UNUSED(B_m);
  assert(A != NULL && B != NULL && C != NULL);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

#if PRECISION == 1
  cblas_sgemm(CblasRowMajor, /* Matrix data arrangement */
              CblasNoTrans,  /* Transpose A */
              CblasNoTrans,  /* Transpose B */
              A_m,           /* Number of rows in A and C */
              B_n,           /* Number of cols in B and C */
              A_n,           /* Number of cols in A */
              1.0,           /* Scaling factor for the product of A and B */
              A,             /* Matrix A */
              A_n,           /* First dimension of A */
              B,             /* Matrix B */
              B_n,           /* First dimension of B */
              0.0,           /* Scale factor for C */
              C,             /* Output */
              B_n);          /* First dimension of C */
#elif PRECISION == 2
  cblas_dgemm(CblasRowMajor, /* Matrix data arrangement */
              CblasNoTrans,  /* Transpose A */
              CblasNoTrans,  /* Transpose B */
              A_m,           /* Number of rows in A and C */
              B_n,           /* Number of cols in B and C */
              A_n,           /* Number of cols in A */
              1.0,           /* Scaling factor for the product of A and B */
              A,             /* Matrix A */
              A_n,           /* First dimension of A */
              B,             /* Matrix B */
              B_n,           /* First dimension of B */
              0.0,           /* Scale factor for C */
              C,             /* Output */
              B_n);          /* First dimension of C */
#endif
}
#endif

/******************************************************************************
 *                                  SVD
 ******************************************************************************/

/* int svd(real_t *A, int m, int n, real_t *U, real_t *s, real_t *V_t) { */
/*   const int lda = n; */
/*   const int ldu = m; */
/*   const int ldvt = n; */
/*   const char jobu = 'A'; */
/*   const char jobvt = 'A'; */
/*   const int superb_size = (m < n) ? m : n; */
/*   real_t *superb = malloc(sizeof(real_t) * (superb_size - 1)); */
/*   int retval = LAPACKE_dgesvd(LAPACK_ROW_MAJOR, */
/*                               jobu, */
/*                               jobvt, */
/*                               m, */
/*                               n, */
/*                               A, */
/*                               lda, */
/*                               s, */
/*                               U, */
/*                               ldu, */
/*                               V_t, */
/*                               ldvt, */
/*                               superb); */
/*   if (retval > 0) { */
/*     return -1; */
/*   } */
/*  */
/*   #<{(| Clean up |)}># */
/*   free(superb); */
/*  */
/*   return 0; */
/* } */

/**
 * svdcomp - SVD decomposition routine.
 * Takes an mxn matrix a and decomposes it into udv, where u,v are
 * left and right orthogonal transformation matrices, and d is a
 * diagonal matrix of singular values.
 *
 * This routine is adapted from svdecomp.c in XLISP-STAT 2.1 which is
 * code from Numerical Recipes adapted by Luke Tierney and David Betz.
 *
 * Input to dsvd is as follows:
 *   A = mxn matrix to be decomposed, gets overwritten with U
 *   m = row dimension of a
 *   n = column dimension of a
 *   w = returns the vector of singular values of a
 *   V = returns the right orthogonal transformation matrix
 */
int svdcomp(real_t *A, int m, int n, real_t *w, real_t *V) {
  /* assert(m < n); */
  int flag, i, its, j, jj, k, l, nm;
  real_t c, f, h, s, x, y, z;
  real_t anorm = 0.0, g = 0.0, scale = 0.0;

  /* Householder reduction to bidiagonal form */
  real_t *rv1 = malloc(sizeof(real_t) * n);
  for (i = 0; i < n; i++) {
    /* left-hand reduction */
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++) {
        scale += fabs(A[k * n + i]);
      }

      if (scale) {
        for (k = i; k < m; k++) {
          A[k * n + i] = (A[k * n + i] / scale);
          s += (A[k * n + i] * A[k * n + i]);
        }

        f = A[i * n + i];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        A[i * n + i] = (f - g);

        if (i != n - 1) {
          for (j = l; j < n; j++) {
            for (s = 0.0, k = i; k < m; k++) {
              s += (A[k * n + i] * A[k * n + j]);
            }
            f = s / h;
            for (k = i; k < m; k++) {
              A[k * n + j] += (f * A[k * n + i]);
            }
          }
        }

        for (k = i; k < m; k++) {
          A[k * n + i] = (A[k * n + i] * scale);
        }
      }
    }
    w[i] = (scale * g);

    /* right-hand reduction */
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++) {
        scale += fabs(A[i * n + k]);
      }

      if (scale) {
        for (k = l; k < n; k++) {
          A[i * n + k] = (A[i * n + k] / scale);
          s += (A[i * n + k] * A[i * n + k]);
        }

        f = A[i * n + l];
        g = -SIGN(sqrt(s), f);
        h = f * g - s;
        A[i * n + l] = (f - g);

        for (k = l; k < n; k++) {
          rv1[k] = A[i * n + k] / h;
        }

        if (i != m - 1) {
          for (j = l; j < m; j++) {
            for (s = 0.0, k = l; k < n; k++) {
              s += (A[j * n + k] * A[i * n + k]);
            }
            for (k = l; k < n; k++) {
              A[j * n + k] += (s * rv1[k]);
            }
          }
        }
        for (k = l; k < n; k++)
          A[i * n + k] = (A[i * n + k] * scale);
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  /* Accumulate the right-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g) {
        for (j = l; j < n; j++) {
          V[j * n + i] = ((A[i * n + j] / A[i * n + l]) / g);
        }
        /* real_t division to avoid underflow */
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++) {
            s += (A[i * n + k] * V[k * n + j]);
          }
          for (k = l; k < n; k++) {
            V[k * n + j] += (s * V[k * n + i]);
          }
        }
      }
      for (j = l; j < n; j++) {
        V[i * n + j] = V[j * n + i] = 0.0;
      }
    }
    V[i * n + i] = 1.0;
    g = rv1[i];
    l = i;
  }

  /* accumulate the left-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    if (i < n - 1) {
      for (j = l; j < n; j++) {
        A[i * n + j] = 0.0;
      }
    }
    if (g) {
      g = 1.0 / g;
      if (i != n - 1) {
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < m; k++) {
            s += (A[k * n + i] * A[k * n + j]);
          }
          f = (s / A[i * n + i]) * g;

          for (k = i; k < m; k++) {
            A[k * n + j] += (f * A[k * n + i]);
          }
        }
      }
      for (j = i; j < m; j++) {
        A[j * n + i] = (A[j * n + i] * g);
      }
    } else {
      for (j = i; j < m; j++) {
        A[j * n + i] = 0.0;
      }
    }
    ++A[i * n + i];
  }

  /* diagonalize the bidiagonal form */
  for (k = n - 1; k >= 0; k--) {     /* loop over singular values */
    for (its = 0; its < 30; its++) { /* loop over allowed iterations */
      flag = 1;
      for (l = k; l >= 0; l--) { /* test for splitting */
        nm = l - 1;
        if (fabs(rv1[l]) + anorm == anorm) {
          flag = 0;
          break;
        }
        if (fabs(w[nm]) + anorm == anorm)
          break;
      }
      if (flag) {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          if (fabs(f) + anorm != anorm) {
            g = w[i];
            h = pythag(f, g);
            w[i] = h;
            h = 1.0 / h;
            c = g * h;
            s = (-f * h);
            for (j = 0; j < m; j++) {
              y = A[j * n + nm];
              z = A[j * n + i];
              A[j * n + nm] = y * c + z * s;
              A[j * n + i] = z * c - y * s;
            }
          }
        }
      }
      z = w[k];
      if (l == k) {    /* convergence */
        if (z < 0.0) { /* make singular value nonnegative */
          w[k] = (-z);
          for (j = 0; j < n; j++)
            V[j * n + k] = (-V[j * n + k]);
        }
        break;
      }
      if (its >= 30) {
        free((void *) rv1);
        fprintf(stderr, "No convergence after 30,000! iterations \n");
        return (0);
      }

      /* Shift from bottom 2 x 2 minor */
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

      /* next QR transformation */
      c = s = 1.0;
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y = y * c;
        for (jj = 0; jj < n; jj++) {
          x = V[(jj * n) + j];
          z = V[(jj * n) + i];
          V[jj * n + j] = x * c + z * s;
          V[jj * n + i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = (c * g) + (s * y);
        x = (c * y) - (s * g);
        for (jj = 0; jj < m; jj++) {
          y = A[jj * n + j];
          z = A[jj * n + i];
          A[jj * n + j] = (y * c + z * s);
          A[jj * n + i] = (z * c - y * s);
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }

  free(rv1);
  return 0;
}

/* int pinv(real_t *A, const int m, const int n, real_t *A_inv) { */
/*   #<{(| Decompose A with SVD |)}># */
/*   real_t *U = malloc(sizeof(real_t) * m * n); */
/*   real_t *d = malloc(sizeof(real_t) * n); */
/*   real_t *V_t = malloc(sizeof(real_t) * n * n); */
/*   if (svd(A, m, n, U, d, V_t) != 0) { */
/*     return -1; */
/*   } */
/*  */
/*   #<{(| Form reciprocal singular matrix S_inv from singular vector d |)}># */
/*   real_t *S_inv = malloc(sizeof(real_t) * n * n); */
/*   zeros(S_inv, n, n); */
/*   int mat_index = 0; */
/*   int vec_index = 0; */
/*   for (int i = 0; i < n; i++) { */
/*     for (int j = 0; j < n; j++) { */
/*       if (i == j) { */
/*         S_inv[mat_index] = 1.0 / d[vec_index]; */
/*         vec_index++; */
/*       } */
/*       mat_index++; */
/*     } */
/*   } */
/*  */
/*   #<{(| pinv(H) = V S^-1 U' |)}># */
/*   real_t *V = malloc(sizeof(real_t) * n * n); */
/*   mat_transpose(V_t, n, n, V); */
/*  */
/*   real_t *U_t = malloc(sizeof(real_t) * n * n); */
/*   mat_transpose(U, n, n, U_t); */
/*  */
/*   real_t *VSi = malloc(sizeof(real_t) * n * n); */
/*   dot(V, n, n, S_inv, n, n, VSi); */
/*   dot(VSi, n, n, U_t, n, n, A_inv); */
/*  */
/*   #<{(| Clean up |)}># */
/*   free(U); */
/*   free(U_t); */
/*   free(d); */
/*   free(S_inv); */
/*   free(V); */
/*   free(V_t); */
/*   free(VSi); */
/*  */
/*   return 0; */
/* } */

/******************************************************************************
 *                                  CHOL
 ******************************************************************************/

void chol(const real_t *A, const size_t n, real_t *L) {
  assert(A != NULL);
  assert(n > 0);
  /* real_t *L = calloc(n * n, sizeof(real_t)); */

  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < (i + 1); j++) {

      if (i == j) {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[j * n + k] * L[j * n + k];
        }
        L[i * n + j] = sqrt(A[i * n + i] - s);

      } else {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[i * n + k] * L[j * n + k];
        }
        L[i * n + j] = (1.0 / L[j * n + j] * (A[i * n + j] - s));
      }
    }
  }
}

void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
  /* Allocate memory */
  real_t *L = calloc(n * n, sizeof(real_t));
  real_t *Lt = calloc(n * n, sizeof(real_t));
  real_t *y = calloc(n, sizeof(real_t));

  /* Cholesky decomposition */
  chol(A, n, L);
  mat_transpose(L, n, n, Lt);

  /* Forward substitution */
  /* Ax = b -> LLt x = b. */
  /* Let y = Lt x, L y = b (Solve for y) */
  for (int i = 0; i < (int) n; i++) {
    real_t alpha = b[i];

    if (fltcmp(L[i * n + i], 0.0) == 0) {
      y[i] = 0.0;

    } else {
      for (int j = 0; j < i; j++) {
        alpha -= L[i * n + j] * y[j];
      }
      y[i] = alpha / L[i * n + i];
    }
  }

  /* Backward substitution */
  /* Now we have y, we can go back to (Lt x = y) and solve for x */
  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];

    if (fltcmp(Lt[i * n + i], 0.0) == 0) {
      x[i] = 0.0;

    } else {
      for (int j = i; j < (int) n; j++) {
        alpha -= Lt[i * n + j] * x[j];
      }
      x[i] = alpha / Lt[i * n + i];
    }
  }

  /* Clean up */
  free(y);
  free(L);
  free(Lt);
}

#ifdef USE_LAPACK
void lapack_chol_solve(const real_t *A,
                       const real_t *b,
                       real_t *x,
                       const size_t m) {
  /* Cholesky Decomposition */
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'L';
  real_t *a = mat_new(m, m);
  mat_copy(A, m, m, a);
#if PRECISION == 1
  spotrf_(&uplo, &n, a, &lda, &info);
#elif PRECISION == 2
  dpotrf_(&uplo, &n, a, &lda, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  /* Solve Ax = b using Cholesky decomposed A from above */
  vec_copy(b, m, x);
  int nhrs = 1;
  int ldb = m;
#if PRECISION == 1
  spotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#elif PRECISION == 2
  dpotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to solve Ax = b!\n");
  }

  free(a);
}
#endif

/******************************************************************************
 *                                   TIME
 ******************************************************************************/

struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

float toc(struct timespec *tic) {
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

float mtoc(struct timespec *tic) { return toc(tic) * 1000.0; }

float time_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((float) t.tv_sec + ((float) t.tv_usec) / 1000000.0);
}

/******************************************************************************
 *                               TRANSFORMS
 ******************************************************************************/

void tf(const real_t params[7], real_t T[4 * 4]) {
  assert(params != NULL);
  assert(T != NULL);

  const real_t q[4] = {params[0], params[1], params[2], params[3]};
  const real_t r[3] = {params[4], params[5], params[6]};

  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[3] = r[0];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[7] = r[1];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
  T[11] = r[2];

  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

void tf_params(const real_t T[4 * 4], real_t params[7]) {
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  real_t r[3] = {0};
  tf_trans_get(T, r);

  real_t q[4] = {0};
  rot2quat(C, q);

  params[0] = q[0];
  params[1] = q[1];
  params[2] = q[2];
  params[3] = q[3];

  params[4] = r[0];
  params[5] = r[1];
  params[6] = r[2];
}

void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
}

void tf_trans_set(real_t T[4 * 4], const real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  T[3] = r[0];
  T[7] = r[1];
  T[11] = r[2];
}

void tf_trans_get(const real_t T[4 * 4], real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];

  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];

  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
}

void tf_quat_get(const real_t T[4 * 4], real_t q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2quat(C, q);
}

void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]) {
  assert(T != NULL);
  assert(T_inv != NULL);
  assert(T != T_inv);

  /* Get original rotation and translation component */
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T, C);
  tf_trans_get(T, r);

  /* Invert rotation component */
  real_t C_inv[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_inv);

  /* Set rotation component */
  tf_rot_set(T_inv, C_inv);

  /* Set translation component */
  real_t r_inv[3] = {0};
  mat_scale(C_inv, 3, 3, -1.0);
  dot(C_inv, 3, 3, r, 3, 1, r_inv);
  tf_trans_set(T_inv, r_inv);

  /* Make sure the last element is 1 */
  T_inv[15] = 1.0;
}

void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]) {
  assert(T != NULL);
  assert(p != NULL);
  assert(retval != NULL);
  assert(p != retval);

  const real_t hp_a[4] = {p[0], p[1], p[2], 1.0};
  real_t hp_b[4] = {0.0, 0.0, 0.0, 0.0};
  dot(T, 4, 4, hp_a, 4, 1, hp_b);

  retval[0] = hp_b[0];
  retval[1] = hp_b[1];
  retval[2] = hp_b[2];
}

void tf_hpoint(const real_t T[4 * 4], const real_t hp[4], real_t retval[4]) {
  assert(T != NULL);
  assert(hp != retval);
  dot(T, 4, 4, hp, 4, 1, retval);
}

void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i) {
  /* Build perturb drvec */
  real_t drvec[3] = {0};
  drvec[i] = step_size;

  /* Decompose transform to rotation and translation */
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  /* Perturb rotation */
  real_t C_rvec[3 * 3] = {0};
  real_t C_diff[3 * 3] = {0};
  rvec2rot(drvec, 1e-8, C_rvec);
  dot(C_rvec, 3, 3, C, 3, 3, C_diff);
  tf_rot_set(T, C_diff);
}

void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i) {
  /* Build perturb dr */
  real_t dr[3] = {0};
  dr[i] = step_size;

  /* Decompose transform get translation */
  real_t r[3] = {0};
  tf_trans_get(T, r);

  /* Perturb translation */
  const real_t r_diff[3] = {r[0] + dr[0], r[1] + dr[1], r[2] + dr[2]};
  tf_trans_set(T, r_diff);
}

void rvec2rot(const real_t *rvec, const real_t eps, real_t *R) {
  /* Magnitude of rvec */
  const real_t theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1]);
  // ^ basically norm(rvec), but faster

  /* Check if rotation is too small */
  if (theta < eps) {
    R[0] = 1.0;
    R[1] = -rvec[2];
    R[2] = rvec[1];

    R[3] = rvec[2];
    R[4] = 1.0;
    R[5] = -rvec[0];

    R[6] = -rvec[1];
    R[7] = rvec[0], R[8] = 1.0;
    return;
  }

  /* Convert rvec to rotation matrix */
  real_t rvec_normed[3] = {rvec[0], rvec[1], rvec[2]};
  vec_scale(rvec_normed, 3, 1 / theta);
  const real_t x = rvec_normed[0];
  const real_t y = rvec_normed[1];
  const real_t z = rvec_normed[2];

  const real_t c = cos(theta);
  const real_t s = sin(theta);
  const real_t C = 1 - c;

  const real_t xs = x * s;
  const real_t ys = y * s;
  const real_t zs = z * s;

  const real_t xC = x * C;
  const real_t yC = y * C;
  const real_t zC = z * C;

  const real_t xyC = x * yC;
  const real_t yzC = y * zC;
  const real_t zxC = z * xC;

  R[0] = x * xC + c;
  R[1] = xyC - zs;
  R[2] = zxC + ys;

  R[3] = xyC + zs;
  R[4] = y * yC + c;
  R[5] = yzC - xs;

  R[6] = zxC - ys;
  R[7] = yzC + xs;
  R[8] = z * zC + c;
}

void euler321(const real_t euler[3], real_t C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const real_t phi = euler[0];
  const real_t theta = euler[1];
  const real_t psi = euler[2];

  /* 1st row */
  C[0] = cos(psi) * cos(theta);
  C[1] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  C[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  /* 2nd row */
  C[3] = sin(psi) * cos(theta);
  C[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  C[5] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  /* 3rd row */
  C[6] = -sin(theta);
  C[7] = cos(theta) * sin(phi);
  C[8] = cos(theta) * cos(phi);
}

void rot2quat(const real_t C[3 * 3], real_t q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const real_t C00 = C[0];
  const real_t C01 = C[1];
  const real_t C02 = C[2];
  const real_t C10 = C[3];
  const real_t C11 = C[4];
  const real_t C12 = C[5];
  const real_t C20 = C[6];
  const real_t C21 = C[7];
  const real_t C22 = C[8];

  const real_t tr = C00 + C11 + C22;
  real_t S = 0.0f;
  real_t qw = 0.0f;
  real_t qx = 0.0f;
  real_t qy = 0.0f;
  real_t qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

void quat2euler(const real_t q[4], real_t euler[3]) {
  assert(q != NULL);
  assert(euler != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qw2 = qw * qw;
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;

  const real_t t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const real_t t2 = asin(2 * (qy * qw - qx * qz));
  const real_t t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  euler[0] = t1;
  euler[1] = t2;
  euler[2] = t3;
}

void quat2rot(const real_t q[4], real_t C[3 * 3]) {
  assert(q != NULL);
  assert(C != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  /* Homogeneous form */
  /* -- 1st row */
  C[0] = qw2 + qx2 - qy2 - qz2;
  C[1] = 2 * (qx * qy - qw * qz);
  C[2] = 2 * (qx * qz + qw * qy);
  /* -- 2nd row */
  C[3] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[5] = 2 * (qy * qz - qw * qx);
  /* -- 3rd row */
  C[6] = 2 * (qx * qz - qw * qy);
  C[7] = 2 * (qy * qz + qw * qx);
  C[8] = qw2 - qx2 - qy2 + qz2;
}

void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);

  const real_t pw = p[0];
  const real_t px = p[1];
  const real_t py = p[2];
  const real_t pz = p[3];

  /* clang-format off */
  const real_t lprod[4*4] = {
    pw, -px, -py, -pz,
    px, pw, -pz, py,
    py, pz, pw, -px,
    pz, -py, px, pw
  };
  /* clang-format on */

  cblas_dot(lprod, 4, 4, q, 4, 1, r);
}

void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  /* clang-format off */
  const real_t rprod[4*4] = {
    qw, -qx, -qy, -qz,
    qx, qw, qz, -qy,
    qy, -qz, qw, qx,
    qz, qy, -qx, qw
  };
  /* clang-format on */

  dot(rprod, 4, 4, p, 4, 1, r);
}

void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL && q != NULL && r != NULL);
  assert(p != r && q != r);
  quat_lmul(p, q, r);
}

void quat_delta(const real_t dalpha[3], real_t dq[4]) {
  const real_t half_norm = 0.5 * vec_norm(dalpha, 3);
  const real_t k = sinc(half_norm) * 0.5;
  const real_t vector[3] = {k * dalpha[0], k * dalpha[1], k * dalpha[2]};
  real_t scalar = cos(half_norm);

  dq[0] = scalar;
  dq[1] = vector[0];
  dq[2] = vector[1];
  dq[3] = vector[2];
}

/*****************************************************************************
 *                                  IMAGE
 *****************************************************************************/

void image_init(image_t *img, uint8_t *data, int width, int height) {
  img->data = data;
  img->width = width;
  img->height = height;
}

/*****************************************************************************
 *                                  CV
 *****************************************************************************/

/********************************* RADTAN ************************************/

void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  /* Distortion parameters */
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  /* Apply tangential distortion */
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

  /* Distorted point */
  p_d[0] = x_ddash;
  p_d[1] = y_ddash;
}

void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]) {
  /* Distortion parameters */
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Point Jacobian is 2x2 */
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x;
  J_point[0] += x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x;
  J_point[3] += y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
}

void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]) {
  UNUSED(params);

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Setup */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t xy = x * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * r2;
  J_param[1] = x * r4;
  J_param[2] = 2 * xy;
  J_param[3] = 3 * x2 + y2;

  J_param[4] = y * r2;
  J_param[5] = y * r4;
  J_param[6] = x2 + 3 * y2;
  J_param[7] = 2 * xy;
}

/********************************** EQUI *************************************/

void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  const real_t x_dash = s * x;
  const real_t y_dash = s * y;

  p_d[0] = x_dash;
  p_d[1] = y_dash;
}

void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]) {
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s = thd / r;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  /* Point Jacobian is 2x2 */
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]) {
  UNUSED(params);

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th3 = th2 * th;
  const real_t th5 = th3 * th2;
  const real_t th7 = th5 * th2;
  const real_t th9 = th7 * th2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * th3 / r;
  J_param[1] = x * th5 / r;
  J_param[2] = x * th7 / r;
  J_param[3] = x * th9 / r;

  J_param[4] = y * th3 / r;
  J_param[5] = y * th5 / r;
  J_param[6] = y * th7 / r;
  J_param[7] = y * th9 / r;
}

/******************************** PINHOLE ************************************/

real_t pinhole_focal(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

int pinhole_project(const real_t params[4], const real_t p_C[3], real_t x[2]) {
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  const real_t px = p_C[0] / p_C[2];
  const real_t py = p_C[1] / p_C[2];

  x[0] = px * fx + cx;
  x[1] = py * fy + cy;

  return 0;
}

void pinhole_point_jacobian(const real_t params[4], real_t J[2 * 3]) {
  J[0] = params[0];
  J[1] = 0.0;
  J[2] = 0.0;
  J[3] = params[1];
}

void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]) {
  UNUSED(params);

  J[0] = x[0];
  J[1] = 0.0;
  J[2] = 1.0;
  J[3] = 0.0;

  J[4] = 0.0;
  J[5] = x[1];
  J[6] = 0.0;
  J[7] = 1.0;
}

/***************************** PINHOLE-RADTAN4 ********************************/

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]) {
  /* Project */
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  /* Distort */
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  /* Scale and center */
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p[0] * fx + cx;
  x[1] = p[1] * fy + cy;
}

void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]) {
  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[0];
  const real_t z = p_C[0];
  const real_t p[2] = {x / z, y / z};

  /* Projection Jacobian */
  real_t J_proj[2 * 3] = {0};
  J_proj[0] = 1.0 / z;
  J_proj[1] = 0.0;
  J_proj[2] = -x / (z * z);
  J_proj[3] = 0.0;
  J_proj[4] = 1.0 / z;
  J_proj[5] = -y / (z * z);

  /* Distortion Point Jacobian */
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_dist_point[2 * 2] = {0};
  radtan4_point_jacobian(d, p, J_dist_point);

  /* Project Point Jacobian */
  real_t J_proj_point[2 * 3] = {0};
  pinhole_point_jacobian(params, J_proj_point);

  /* J = J_proj * J_dist_point * J_proj_point; */
  real_t J_dist_proj[2 * 3] = {0};
  dot(J_dist_point, 2, 2, J_proj, 2, 3, J_dist_proj);
  dot(J_proj, 2, 2, J_dist_proj, 2, 3, J);
}

void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]) {
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[0];
  const real_t z = p_C[0];
  const real_t p[2] = {x / z, y / z};

  /* Distort */
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  /* Project params Jacobian: J_proj_params */
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  /* Project point Jacobian: J_proj_point */
  real_t J_proj_point[2 * 3] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  /* Distortion point Jacobian: J_dist_params */
  real_t J_dist_params[2 * 2] = {0};
  radtan4_params_jacobian(d, p, J_dist_params);

  /* J = [J_proj_params, J_proj_point * J_dist_params] */
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];


}

/****************************** PINHOLE-EQUI4 *********************************/

void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]) {
  /* Project */
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  /* Distort */
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  /* Scale and center */
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p[0] * fx + cx;
  x[1] = p[1] * fy + cy;
}

void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]) {
  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[0];
  const real_t z = p_C[0];
  const real_t p[2] = {x / z, y / z};

  /* Projection Jacobian */
  real_t J_proj[2 * 3] = {0};
  J_proj[0] = 1.0 / z;
  J_proj[1] = 0.0;
  J_proj[2] = -x / (z * z);
  J_proj[3] = 0.0;
  J_proj[4] = 1.0 / z;
  J_proj[5] = -y / (z * z);

  /* Distortion Point Jacobian */
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_dist_point[2 * 2] = {0};
  equi4_point_jacobian(d, p, J_dist_point);

  /* Project Point Jacobian */
  real_t J_proj_point[2 * 3] = {0};
  pinhole_point_jacobian(params, J_proj_point);

  /* J = J_proj * J_dist_point * J_proj_point; */
  real_t J_dist_proj[2 * 3] = {0};
  dot(J_dist_point, 2, 2, J_proj, 2, 3, J_dist_proj);
  dot(J_proj, 2, 2, J_dist_proj, 2, 3, J);
}
