#include "util.hpp"

/*****************************************************************************
 *                                  DATA
 *****************************************************************************/

int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

int32_t sint32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) | (data[offset]));
}

uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | (data[offset]));
}

char *malloc_string(const char *s) {
  char *retval = (char *) malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
}

int csv_rows(const char *fp) {
  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return -1;
  }

  // Loop through lines
  int nb_rows = 0;
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  // Cleanup
  fclose(infile);

  return nb_rows;
}

int csv_cols(const char *fp) {
  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return -1;
  }

  // Get line that isn't the header
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  // Parse line to obtain number of elements
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < len_max; i++) {
    if (line[i] == ',') {
      found_separator = 1;
      nb_elements++;
    }
  }

  // Cleanup
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

char **csv_fields(const char *fp, int *nb_fields) {
  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Get last header line
  char field_line[1024] = {0};
  char line[1024] = {0};
  size_t len_max = 1024;
  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      strcpy(field_line, line);
    }
  }

  // Parse fields
  *nb_fields = csv_cols(fp);
  char **fields = (char **) malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    // Ignore # and ' '
    if (c == '#' || c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      // Add field name to fields
      fields[field_idx] = malloc_string(field_name);
      memset(field_name, '\0', sizeof(char) * 100);
      field_idx++;
    } else {
      // Append field name
      field_name[strlen(field_name)] = c;
    }
  }

  // Cleanup
  fclose(infile);

  return fields;
}

real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  // Obtain number of rows and columns in csv data
  *nb_rows = csv_rows(fp);
  *nb_cols = csv_cols(fp);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  // Initialize memory for csv data
  real_t **data = (real_t **) malloc(sizeof(real_t *) * *nb_rows);
  for (int i = 0; i < *nb_cols; i++) {
    data[i] = (real_t *) malloc(sizeof(real_t) * *nb_cols);
  }

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Loop through data
  char line[1024] = {0};
  size_t len_max = 1024;
  int row_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

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

  // Cleanup
  fclose(infile);

  return data;
}

static int *parse_iarray_line(char *line) {
  char entry[1024] = {0};
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
        data = (int *) calloc(array_size + 1, sizeof(int));
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
  *nb_arrays = csv_rows(csv_path);
  int **array = (int **) calloc(*nb_arrays, sizeof(int *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
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
  char entry[1024] = {0};
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
        data = (real_t *) calloc(array_size, sizeof(real_t));
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
  *nb_arrays = csv_rows(csv_path);
  real_t **array = (real_t **) calloc(*nb_arrays, sizeof(real_t *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_darray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

int csv_rows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int csv_cols(const std::string &file_path) {
  int nb_elements = 1;
  bool found_separator = false;

  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of commas
  std::string line;
  std::getline(infile, line);
  for (size_t i = 0; i < line.length(); i++) {
    if (line[i] == ',') {
      found_separator = true;
      nb_elements++;
    }
  }

  return (found_separator) ? nb_elements : 0;
}

int csv2mat(const std::string &file_path, const bool header, matx_t &data) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of rows and cols
  int nb_rows = csv_rows(file_path);
  int nb_cols = csv_cols(file_path);

  // Skip header line?
  std::string line;
  if (header) {
    std::getline(infile, line);
    nb_rows -= 1;
  }

  // Load data
  int line_no = 0;
  std::vector<real_t> vdata;
  data = zeros(nb_rows, nb_cols);

  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    // Load data row
    std::string element;
    for (int i = 0; i < nb_cols; i++) {
      std::getline(ss, element, ',');
      const real_t value = atof(element.c_str());
      data(line_no, i) = value;
    }

    line_no++;
  }

  return 0;
}

int mat2csv(const std::string &file_path, const matx_t &data) {
	const auto precision = Eigen::FullPrecision;
	const auto alignment = Eigen::DontAlignCols;
	const Eigen::IOFormat CSVFormat(precision, alignment, ",", "\n");

	std::ofstream csv_file(file_path);
	if (csv_file.is_open() != true) {
		return -1;
	}

	csv_file << data.format(CSVFormat);
	csv_file.close();

  return 0;
}

int vec2csv(const std::string &file_path, const std::deque<vec3_t> &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save vector
  for (const auto &v : data) {
    outfile << v(0);
    outfile << ",";
    outfile << v(1);
    outfile << ",";
    outfile << v(2);
    outfile << std::endl;
  }

  // Close file
  outfile.close();
  return 0;
}

int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data) {
  // Open file
  std::ofstream outfile(file_path);
  if (outfile.good() != true) {
    return -1;
  }

  // Save vector
  for (const auto &ts : data) {
    outfile << ts << std::endl;
  }

  // Close file
  outfile.close();
  return 0;
}

void print_progress(const real_t percentage) {
  const char *PBSTR =
      "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
  const int PBWIDTH = 60;

  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);

  if ((fabs(percentage - 1.0) < 1e-10)) {
    printf("\n");
  }
}

bool all_true(const std::vector<bool> x) {
  for (const auto i : x) {
    if (i == false) {
      return false;
    }
  }

  return true;
}

int check_jacobian(const std::string &jac_name,
                   const matx_t &fdiff,
                   const matx_t &jac,
                   const real_t threshold,
                   const bool print) {
  // Pre-check
  if (jac.size() == 0) {
    LOG_ERROR("Provided analytical jacobian is empty!");
    return false;
  } else if (fdiff.size() == 0) {
    LOG_ERROR("Provided numerical jacobian is empty!");
    return false;
  } else if (fdiff.rows() != jac.rows()) {
    LOG_ERROR("rows(fdiff) != rows(jac)");
    return false;
  } else if (fdiff.cols() != jac.cols()) {
    LOG_ERROR("cols(fdiff) != cols(jac)");
    return false;
  }

  // Check if any of the values are beyond the threshold
  const matx_t delta = (fdiff - jac);
  bool failed = false;
  for (long i = 0; i < delta.rows(); i++) {
    for (long j = 0; j < delta.cols(); j++) {
      if (fabs(delta(i, j)) >= threshold) {
        failed = true;
      }
    }
  }

  // Print result
  int retval = 0;
  if (failed) {
    if (print) {
      LOG_ERROR("Check [%s] failed!\n", jac_name.c_str());
      print_matrix("num diff jac", fdiff);
      print_matrix("analytical jac", jac);
      print_matrix("difference matrix", delta);
      // exit(-1);
    }
    retval = -1;

  } else {
    if (print) {
      printf("Check [%s] passed!\n", jac_name.c_str());
    }
    retval = 0;
  }

  return retval;
}

/******************************************************************************
 *                               FILESYSTEM
 *****************************************************************************/

FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows) {
  FILE *fp = fopen(path.c_str(), mode.c_str());
  if (fp == NULL) {
    return nullptr;
  }

  if (nb_rows != nullptr) {
    *nb_rows = file_rows(path);
  }

  return fp;
}

void skip_line(FILE *fp) {
  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

int file_rows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int file_copy(const std::string &src, const std::string &dest) {
  // Open input path
  FILE *src_file = fopen(src.c_str(), "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  // Open output path
  FILE *dest_file = fopen(dest.c_str(), "wb");
  if (dest_file == NULL) {
    fclose(src_file);
    fclose(dest_file);
    return -2;
  }

  // BUFSIZE default is 8192 bytes
  // BUFSIZE of 1 means one chareter at time
  char buf[BUFSIZ];
  while (size_t size = fread(buf, 1, BUFSIZ, src_file)) {
    fwrite(buf, 1, size, dest_file);
  }

  // Clean up
  fclose(src_file);
  fclose(dest_file);

  return 0;
}

std::string parse_fext(const std::string &path) {
  return path.substr(path.find_last_of("."));
}

std::string parse_fname(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
}

bool file_exists(const std::string &path) {
  FILE *file;

  file = fopen(path.c_str(), "r");
  if (file != NULL) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool dir_exists(const std::string &path) {
  DIR *dir = opendir(path.c_str());

  if (dir) {
    closedir(dir);
    return true;
  } else if (ENOENT == errno) {
    return false;
  } else {
    LOG_ERROR("dir_exists() failed! %s", strerror(errno));
    exit(-1);
  }
}

int dir_create(const std::string &path) {
  std::string command = "mkdir -p " + path;
  return system(command.c_str());
}

std::string dir_name(const std::string &path) {
  std::size_t found = path.find_last_of("/\\");
  return path.substr(0, found);
}

std::string strip(const std::string &s, const std::string &target) {
  size_t first = s.find_first_not_of(target);
  if (std::string::npos == first) {
    return s;
  }

  size_t last = s.find_last_not_of(target);
  return s.substr(first, (last - first + 1));
}

std::string strip_end(const std::string &s, const std::string &target) {
  size_t last = s.find_last_not_of(target);
  return s.substr(0, last + 1);
}

int create_dir(const std::string &path) {
  const std::string command = "mkdir -p " + path;
  const int retval = system(command.c_str());
  if (retval == -1) {
    printf("Error creating directory!n");
    return -1;
  }

  return 0;
}

int remove_dir(const std::string &path) {
  DIR *dir = opendir(path.c_str());
  struct dirent *next_file;

  // pre-check
  if (dir == NULL) {
    return -1;
  }

  // remove files in path
  while ((next_file = readdir(dir)) != NULL) {
    remove(std::string(path + "/" + next_file->d_name).c_str());
  }

  // remove dir
  remove(path.c_str());
  closedir(dir);

  return 0;
}

std::string remove_ext(const std::string &path) {
  auto output = path;
  const size_t period_idx = output.rfind('.');
  if (std::string::npos != period_idx) {
    output.erase(period_idx);
  }
  return output;
}

int list_dir(const std::string &path, std::vector<std::string> &results) {
  struct dirent *entry;
  DIR *dp;

  // Check directory
  dp = opendir(path.c_str());
  if (dp == NULL) {
    return -1;
  }

  // List directory
  while ((entry = readdir(dp))) {
    std::string value(entry->d_name);
    if (value != "." && value != "..") {
      results.push_back(value);
    }
  }

  // Clean up
  closedir(dp);
  return 0;
}

std::vector<std::string> path_split(const std::string path) {
  std::string s;
  std::vector<std::string> splits;

  s = "";
  for (size_t i = 0; i < path.length(); i++) {
    if (s != "" && path[i] == '/') {
      splits.push_back(s);
      s = "";
    } else if (path[i] != '/') {
      s += path[i];
    }
  }
  splits.push_back(s);

  return splits;
}

std::string paths_combine(const std::string path1, const std::string path2) {
  int dirs_up;
  std::string result;
  std::vector<std::string> splits1;
  std::vector<std::string> splits2;

  // setup
  result = "";
  splits1 = path_split(path1);
  splits2 = path_split(path2);

  // obtain number of directory ups in path 2
  dirs_up = 0;
  for (size_t i = 0; i < splits2.size(); i++) {
    if (splits2[i] == "..") {
      dirs_up++;
    }
  }

  // drop path1 elements as path2 dir ups
  for (int i = 0; i < dirs_up; i++) {
    splits1.pop_back();
  }

  // append path1 to result
  if (path1[0] == '/') {
    result += "/";
  }
  for (size_t i = 0; i < splits1.size(); i++) {
    result += splits1[i];
    result += "/";
  }

  // append path2 to result
  for (size_t i = dirs_up; i < splits2.size(); i++) {
    result += splits2[i];
    result += "/";
  }

  // remove trailing slashes
  for (size_t i = result.length() - 1; i > 0; i--) {
    if (result[i] == '/') {
      result.pop_back();
    } else {
      break;
    }
  }

  return result;
}

/******************************************************************************
 *                                ALGEBRA
 *****************************************************************************/

int sign(const real_t x) {
  if (fltcmp(x, 0.0) == 0) {
    return 0;
  } else if (x < 0) {
    return -1;
  }
  return 1;
}

int fltcmp(const real_t f1, const real_t f2) {
  if (fabs(f1 - f2) <= 0.0001) {
    return 0;
  } else if (f1 > f2) {
    return 1;
  } else {
    return -1;
  }
}

real_t binomial(const real_t n, const real_t k) {
  if (k == 0 || k == n) {
    return 1.0;
  }

  return binomial(n - 1, k - 1) + binomial(n - 1, k);
}

/******************************************************************************
 *                            LINEAR ALGEBRA
 *****************************************************************************/

void print_shape(const std::string &name, const matx_t &A) {
  std::cout << name << ": " << A.rows() << "x" << A.cols() << std::endl;
}

void print_shape(const std::string &name, const vecx_t &v) {
  std::cout << name << ": " << v.rows() << "x" << v.cols() << std::endl;
}

void print_array(const std::string &name,
                 const real_t *array,
                 const size_t size) {
  std::cout << name << std::endl;
  for (size_t i = 0; i < size; i++) {
    printf("%.4f ", array[i]);
  }
  printf("\b\n");
}

void print_vector(const std::string &name, const vecx_t &v) {
  printf("%s: ", name.c_str());
  for (long i = 0; i < v.size(); i++) {
    printf("%f", v(i));
    if ((i + 1) != v.size()) {
      printf(", ");
    }
  }
  printf("\n");
}

void print_matrix(const std::string &name, const matx_t &m) {
  printf("%s:\n", name.c_str());
  for (long i = 0; i < m.rows(); i++) {
    for (long j = 0; j < m.cols(); j++) {
      printf("%f", m(i, j));
      if ((j + 1) != m.cols()) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void print_quaternion(const std::string &name, const quat_t &q) {
  printf("%s: ", name.c_str());
  printf("w:%f, x:%f, y:%f, z:%f\n", q.w(), q.x(), q.y(), q.z());
}

std::string array2str(const real_t *array, const size_t size) {
  std::stringstream os;
  for (size_t i = 0; i < (size - 1); i++) {
    os << array[i] << " ";
  }
  os << array[size - 1];

  return os.str();
}

void array2vec(const real_t *x, const size_t size, vecx_t y) {
  y.resize(size);
  for (size_t i = 0; i < size; i++) {
    y(i) = x[i];
  }
}

real_t *vec2array(const vecx_t &v) {
  real_t *array = (real_t *) malloc(sizeof(real_t) * v.size());
  for (int i = 0; i < v.size(); i++) {
    array[i] = v(i);
  }
  return array;
}

real_t *mat2array(const matx_t &m) {
  real_t *array = (real_t *) malloc(sizeof(real_t) * m.size());

  int index = 0;
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      array[index] = m(i, j);
      index++;
    }
  }
  return array;
}

real_t *quat2array(const quat_t &q) {
  real_t *array = (real_t *) malloc(sizeof(real_t) * 4);

  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();

  return array;
}

void vec2array(const vecx_t &v, real_t *out) {
  for (int i = 0; i < v.size(); i++) {
    out[i] = v(i);
  }
}

void mat2array(const matx_t &A, real_t *out) {
  int index = 0;
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      out[index] = A(i, j);
      index++;
    }
  }
}

std::vector<vecx_t> mat2vec(const matx_t &m, bool row_wise) {
  std::vector<vecx_t> vectors;

  if (row_wise) {
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

vec3s_t mat2vec3(const matx_t &m, bool row_wise) {
  vec3s_t vectors;

  if (row_wise) {
    assert(m.cols() == 3);
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    assert(m.rows() == 3);
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

vec2s_t mat2vec2(const matx_t &m, bool row_wise) {
  vec2s_t vectors;

  if (row_wise) {
    assert(m.cols() == 2);
    for (long i = 0; i < m.rows(); i++) {
      vectors.emplace_back(m.row(i));
    }
  } else {
    assert(m.rows() == 2);
    for (long i = 0; i < m.cols(); i++) {
      vectors.emplace_back(m.col(i));
    }
  }

  return vectors;
}

matx_t vecs2mat(const vec3s_t &vs) {
  matx_t retval;
  retval.resize(4, vs.size());

  int idx = 0;
  for (const auto &v : vs) {
    const real_t x = v(0);
    const real_t y = v(1);
    const real_t z = v(2);
    retval.block(0, idx, 4, 1) = vec4_t{x, y, z, 1.0};
    idx++;
  }

  return retval;
}

std::string vec2str(const vecx_t &v, bool brackets) {
  std::string str;

  if (brackets) {
    str += "[";
  }

  for (int i = 0; i < v.size(); i++) {
    str += std::to_string(v(i));
    if ((i + 1) != v.size()) {
      str += ", ";
    }
  }

  if (brackets) {
    str += "]";
  }

  return str;
}

std::string arr2str(const real_t *arr, const size_t len, bool brackets) {
  std::string str;

  if (brackets) {
    str += "[";
  }

  for (size_t i = 0; i < len; i++) {
    str += std::to_string(arr[i]);
    if ((i + 1) != len) {
      str += ", ";
    }
  }

  if (brackets) {
    str += "]";
  }

  return str;
}

std::string mat2str(const matx_t &m, const std::string &indent) {
  std::string str;

  for (int i = 0; i < m.rows(); i++) {
    if ((i + 1) != m.rows()) {
      str += indent;
      str += vec2str(m.row(i), false) + ",\n";
    } else {
      str += indent;
      str += vec2str(m.row(i), false);
    }
  }

  return str;
}

vec3_t normalize(const vec3_t &v) { return v / v.norm(); }

real_t cond(const matx_t &A) {
  Eigen::JacobiSVD<matx_t> svd(A);
  const auto max_sigma = svd.singularValues()(0);
  const auto min_sigma = svd.singularValues()(svd.singularValues().size() - 1);
  return max_sigma / min_sigma;
}

matx_t zeros(const int rows, const int cols) {
  return matx_t::Zero(rows, cols);
}

matx_t zeros(const int size) { return matx_t::Zero(size, size); }

matx_t I(const int rows, const int cols) {
  return matx_t::Identity(rows, cols);
}

matx_t I(const int size) { return matx_t::Identity(size, size); }

matx_t ones(const int rows, const int cols) {
  matx_t A{rows, cols};
  A.fill(1.0);
  return A;
}

matx_t ones(const int size) { return ones(size, size); }

matx_t hstack(const matx_t &A, const matx_t &B) {
  matx_t C(A.rows(), A.cols() + B.cols());
  C << A, B;
  return C;
}

matx_t vstack(const matx_t &A, const matx_t &B) {
  matx_t C(A.rows() + B.rows(), A.cols());
  C << A, B;
  return C;
}

matx_t dstack(const matx_t &A, const matx_t &B) {
  matx_t C = zeros(A.rows() + B.rows(), A.cols() + B.cols());
  C.block(0, 0, A.rows(), A.cols()) = A;
  C.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
  return C;
}

mat3_t skew(const vec3_t &w) {
  mat3_t S;
  // clang-format off
  S << 0.0, -w(2), w(1),
       w(2), 0.0, -w(0),
       -w(1), w(0), 0.0;
  // clang-format on
  return S;
}

mat3_t skewsq(const vec3_t &w) {
  mat3_t SS = (w * w.transpose()) - pow(w.norm(), 2) * I(3);
  return SS;
}

matx_t enforce_psd(const matx_t &A) {
  matx_t A_psd;

  A_psd.resize(A.rows(), A.cols());

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i == j) {
        A_psd(i, j) = std::fabs(A(i, j));
      } else {
        const real_t x = 0.5 * (A(i, j) + A(j, i));
        A_psd(i, j) = x;
        A_psd(j, i) = x;
      }
    }
  }

  return A_psd;
}

matx_t nullspace(const matx_t &A) {
  Eigen::FullPivLU<matx_t> lu(A);
  matx_t A_null_space = lu.kernel();
  return A_null_space;
}

void load_matrix(const std::vector<real_t> &x,
                 const int rows,
                 const int cols,
                 matx_t &y) {
  int idx;

  // Setup
  idx = 0;
  y.resize(rows, cols);

  // Load matrix
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      y(j, i) = x[idx];
      idx++;
    }
  }
}

void load_matrix(const matx_t &A, std::vector<real_t> &x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) {
      x.push_back(A(j, i));
    }
  }
}

void schurs_complement(const matx_t &H, const vecx_t &b,
                       const size_t m, const size_t r,
                       matx_t &H_marg, vecx_t &b_marg,
                       const bool precond, const bool debug) {
  assert(m > 0 && r > 0);

  // Setup
  const long local_size = m + r;
  H_marg = zeros(local_size, local_size);
  b_marg = zeros(local_size, 1);

  // Precondition Hmm
  matx_t Hmm = H.block(0, 0, m, m);
  if (precond) {
    Hmm = 0.5 * (Hmm + Hmm.transpose());
  }

  // Pseudo inverse of Hmm via Eigen-decomposition:
  //
  //   A_pinv = V * Lambda_pinv * V_transpose
  //
  // Where Lambda_pinv is formed by **replacing every non-zero diagonal entry
  // by its reciprocal, leaving the zeros in place, and transposing the
  // resulting matrix.**
  //
  // clang-format off
  const double eps = 1.0e-8;
  const Eigen::SelfAdjointEigenSolver<matx_t> eig(Hmm);
  const matx_t V = eig.eigenvectors();
  const auto eigvals = eig.eigenvalues().array();
  const auto eigvals_inv = (eigvals > eps).select(eigvals.inverse(), 0);
  const matx_t Lambda_inv = vecx_t(eigvals_inv).asDiagonal();
  const matx_t Hmm_inv = V * Lambda_inv * V.transpose();
  // clang-format on

  // Calculate Schur's complement
  const matx_t Hmr = H.block(0, m, m, r);
  const matx_t Hrm = H.block(m, 0, r, m);
  const matx_t Hrr = H.block(m, m, r, r);
  const vecx_t bmm = b.segment(0, m);
  const vecx_t brr = b.segment(m, r);
  H_marg = Hrr - Hrm * Hmm_inv * Hmr;
  b_marg = brr - Hrm * Hmm_inv * bmm;
  const double inv_check = ((Hmm * Hmm_inv) - I(m, m)).sum();
  if (fabs(inv_check) > 1e-4) {
    LOG_ERROR("FAILED!: Inverse identity check: %f", inv_check);
  }

  if (debug) {
    mat2csv("/tmp/H.csv", H);
    mat2csv("/tmp/Hmm.csv", Hmm);
    mat2csv("/tmp/Hmr.csv", Hmr);
    mat2csv("/tmp/Hrm.csv", Hrm);
    mat2csv("/tmp/Hrr.csv", Hrr);
    mat2csv("/tmp/bmm.csv", bmm);
    mat2csv("/tmp/brr.csv", brr);
    mat2csv("/tmp/H_marg.csv", H_marg);
    mat2csv("/tmp/b_marg.csv", b_marg);
  }
}

/******************************************************************************
 *                                GEOMETRY
 *****************************************************************************/

real_t sinc(const real_t x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const real_t c_2 = 1.0 / 6.0;
    static const real_t c_4 = 1.0 / 120.0;
    static const real_t c_6 = 1.0 / 5040.0;
    const real_t x_2 = x * x;
    const real_t x_4 = x_2 * x_2;
    const real_t x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

vec3_t deg2rad(const vec3_t d) { return d * (M_PI / 180.0); }

real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

vec3_t rad2deg(const vec3_t &r) { return r * (180.0 / M_PI); }

real_t wrap180(const real_t euler_angle) {
  return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

real_t wrap360(const real_t euler_angle) {
  if (euler_angle > 0) {
    return fmod(euler_angle, 360.0);
  } else {
    return fmod(euler_angle + 360, 360.0);
  }
}

real_t wrapPi(const real_t r) { return deg2rad(wrap180(rad2deg(r))); }

real_t wrap2Pi(const real_t r) { return deg2rad(wrap360(rad2deg(r))); }

vec2_t circle(const real_t r, const real_t theta) {
  return vec2_t{r * cos(theta), r * sin(theta)};
}

vec3_t sphere(const real_t rho, const real_t theta, const real_t phi) {
  const real_t x = rho * sin(theta) * cos(phi);
  const real_t y = rho * sin(theta) * sin(phi);
  const real_t z = rho * cos(theta);
  return vec3_t{x, y, z};
}

mat4_t lookat(const vec3_t &cam_pos,
              const vec3_t &target,
              const vec3_t &up_axis) {
  // Note: If we were using OpenGL the cam_dir would be the opposite direction,
  // since in OpenGL the camera forward is -z. In robotics however our camera
  // is +z forward.
  const vec3_t cam_dir = (target - cam_pos).normalized();
  const vec3_t cam_right = (up_axis.cross(cam_dir)).normalized();
  const vec3_t cam_up = cam_dir.cross(cam_right);

  // clang-format off
  mat4_t A;
  A << cam_right(0), cam_right(1), cam_right(2), 0.0,
       cam_up(0), cam_up(1), cam_up(2), 0.0,
       cam_dir(0), cam_dir(1), cam_dir(2), 0.0,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // clang-format off
  mat4_t B;
  B << 1.0, 0.0, 0.0, -cam_pos(0),
       0.0, 1.0, 0.0, -cam_pos(1),
       0.0, 0.0, 1.0, -cam_pos(2),
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  mat4_t T_camera_target = A * B;
  mat4_t T_target_camera = T_camera_target.inverse();
  return T_target_camera;
}

real_t cross_track_error(const vec2_t &p1,
                         const vec2_t &p2,
                         const vec2_t &pos) {
  const real_t x0 = pos(0);
  const real_t y0 = pos(1);

  const real_t x1 = p1(0);
  const real_t y1 = p1(0);

  const real_t x2 = p2(0);
  const real_t y2 = p2(0);

  // calculate perpendicular distance between line (p1, p2) and point (pos)
  const real_t n = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  const real_t d = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

  return fabs(n) / d;
}

int point_left_right(const vec2_t &a, const vec2_t &b, const vec2_t &c) {
  const real_t a0 = a(0);
  const real_t a1 = a(1);
  const real_t b0 = b(0);
  const real_t b1 = b(1);
  const real_t c0 = c(0);
  const real_t c1 = c(1);
  const real_t x = (b0 - a0) * (c1 - a1) - (b1 - a1) * (c0 - a0);

  if (x > 0) {
    return 1; // left
  } else if (x < 0) {
    return 2; // right
  } else if (x == 0) {
    return 0; // parallel
  }

  return -1;
}

real_t closest_point(const vec2_t &a,
                     const vec2_t &b,
                     const vec2_t &p,
                     vec2_t &closest) {
  // pre-check
  if ((a - b).norm() == 0) {
    closest = a;
    return -1;
  }

  // calculate closest point
  const vec2_t v1 = p - a;
  const vec2_t v2 = b - a;
  const real_t t = v1.dot(v2) / v2.squaredNorm();
  closest = a + t * v2;

  return t;
}

void latlon_offset(real_t lat_ref,
                   real_t lon_ref,
                   real_t offset_N,
                   real_t offset_E,
                   real_t *lat_new,
                   real_t *lon_new) {
  *lat_new = lat_ref + (offset_E / EARTH_RADIUS_M);
  *lon_new = lon_ref + (offset_N / EARTH_RADIUS_M) / cos(deg2rad(lat_ref));
}

void latlon_diff(real_t lat_ref,
                 real_t lon_ref,
                 real_t lat,
                 real_t lon,
                 real_t *dist_N,
                 real_t *dist_E) {
  real_t d_lon = lon - lon_ref;
  real_t d_lat = lat - lat_ref;

  *dist_N = deg2rad(d_lat) * EARTH_RADIUS_M;
  *dist_E = deg2rad(d_lon) * EARTH_RADIUS_M * cos(deg2rad(lat));
}

real_t latlon_dist(real_t lat_ref, real_t lon_ref, real_t lat, real_t lon) {
  real_t dist_N = 0.0;
  real_t dist_E = 0.0;

  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  real_t dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));

  return dist;
}

/*****************************************************************************
 *                         DIFFERENTIAL GEOMETRY
 *****************************************************************************/

namespace lie {

mat3_t Exp(const vec3_t &phi) {
  const real_t norm = phi.norm();

  // Small angle approx
  if (norm < 1e-3) {
    return mat3_t{I(3) + skew(phi)};
  }

  // Exponential map from so(3) to SO(3)
  const mat3_t phi_skew = skew(phi);
  mat3_t C = I(3);
  C += (sin(norm) / norm) * phi_skew;
  C += ((1 - cos(norm)) / (norm * norm)) * (phi_skew * phi_skew);

  return C;
}

// vec3_t Log(const mat3_t &C) {
//   const auto phi = acos(C.trace() - 1 / 2);
//   return phi * (C * C.transpose()) / (2 * sin(phi));
// }

mat3_t Jr(const vec3_t &psi) {
  const real_t psi_norm = psi.norm();
  const real_t psi_norm_sq = psi_norm * psi_norm;
  const real_t psi_norm_cube = psi_norm_sq * psi_norm;
  const mat3_t psi_skew = skew(psi);
  const mat3_t psi_skew_sq = psi_skew * psi_skew;

  mat3_t J = I(3);
  J -= ((1 - cos(psi_norm)) / psi_norm_sq) * psi_skew;
  J += (psi_norm - sin(psi_norm)) / (psi_norm_cube) * psi_skew_sq;
  return J;
}

} // namespace lie


/******************************************************************************
 *                               STATISTICS
 *****************************************************************************/

int randi(int ub, int lb) { return rand() % lb + ub; }

real_t randf(const real_t ub, const real_t lb) {
  const real_t f = (real_t) rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

real_t median(const std::vector<real_t> &v) {
  // sort values
  std::vector<real_t> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const real_t a = v_copy[v_copy.size() / 2];
    const real_t b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

vec3_t mean(const vec3s_t &x) {
  vec3_t x_hat{0.0, 0.0, 0.0};

  for (const auto &v : x) {
    x_hat += v;
  }
  x_hat *= 1.0f / x.size();

  return x_hat;
}

real_t shannon_entropy(const matx_t &covar) {
  const real_t n = covar.rows();
  const real_t covar_det = covar.determinant();
  const real_t entropy = 0.5 * log(pow(2 * M_PI * exp(1), n) * covar_det);
  return entropy;
}

vec3_t mvn(std::default_random_engine &engine,
           const vec3_t &mu,
           const vec3_t &stdev) {
  std::normal_distribution<real_t> normal_x(mu(0), stdev(0));
  std::normal_distribution<real_t> normal_y(mu(1), stdev(1));
  std::normal_distribution<real_t> normal_z(mu(2), stdev(2));
  return vec3_t{normal_x(engine), normal_y(engine), normal_z(engine)};
}

real_t gauss_normal() {
  static real_t V1, V2, S;
  static int phase = 0;
  real_t X;

  if (phase == 0) {
    do {
      real_t U1 = (real_t) rand() / RAND_MAX;
      real_t U2 = (real_t) rand() / RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    } while (S >= 1 || S == 0);

    X = V1 * sqrt(-2 * log(S) / S);
  } else {
    X = V2 * sqrt(-2 * log(S) / S);
  }

  phase = 1 - phase;
  return X;
}

/*****************************************************************************
 *                               TRANSFORM
 *****************************************************************************/

mat4_t tf(const vecx_t &params) {
  assert(params.size() == 7);
  const quat_t q{params(0), params(1), params(2), params(3)};
  const vec3_t r{params(4), params(5), params(6)};
  return tf(q, r);
}

mat4_t tf(const mat3_t &C, const vec3_t &r) {
  mat4_t T = I(4);
  T.block(0, 0, 3, 3) = C;
  T.block(0, 3, 3, 1) = r;
  return T;
}

mat4_t tf(const quat_t &q, const vec3_t &r) {
  return tf(q.toRotationMatrix(), r);
}

mat4_t tf_perturb_rot(const mat4_t &T, real_t step_size, const int i) {
  const mat3_t drvec = I(3) * step_size;
  const mat3_t C = tf_rot(T);
  const vec3_t r = tf_trans(T);
  const mat3_t C_diff = rvec2rot(drvec.col(i), 1e-8) * C;
  return tf(C_diff, r);
}

mat4_t tf_perturb_trans(const mat4_t &T, const real_t step_size, const int i) {
  const mat3_t dr = I(3) * step_size;
  const mat3_t C = tf_rot(T);
  const vec3_t r = tf_trans(T);
  const vec3_t r_diff = r + dr.col(i);
  return tf(C, r_diff);
}

vec3_t tf_point(const mat4_t &T, const vec3_t &p) {
  return (T * p.homogeneous()).head(3);
}

mat3_t rotx(const real_t theta) {
  mat3_t R;

  // clang-format off
  R << 1.0, 0.0, 0.0,
       0.0, cos(theta), sin(theta),
       0.0, -sin(theta), cos(theta);
  // clang-format on

  return R;
}

mat3_t roty(const real_t theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), 0.0, -sin(theta),
       0.0, 1.0, 0.0,
       sin(theta), 0.0, cos(theta);
  // clang-format on

  return R;
}

mat3_t rotz(const real_t theta) {
  mat3_t R;

  // clang-format off
  R << cos(theta), sin(theta), 0.0,
       -sin(theta), cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on

  return R;
}

mat3_t euler123(const vec3_t &euler) {
  // i.e. XYZ rotation sequence (body to world)
  const real_t phi = euler(0);
  const real_t theta = euler(1);
  const real_t psi = euler(2);

  const real_t R11 = cos(psi) * cos(theta);
  const real_t R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const real_t R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);

  const real_t R12 = sin(psi) * cos(theta);
  const real_t R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const real_t R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);

  const real_t R13 = -sin(theta);
  const real_t R23 = cos(theta) * sin(phi);
  const real_t R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

mat3_t euler321(const vec3_t &euler) {
  // i.e. ZYX rotation sequence (world to body)
  const real_t phi = euler(0);
  const real_t theta = euler(1);
  const real_t psi = euler(2);

  const real_t R11 = cos(psi) * cos(theta);
  const real_t R21 = sin(psi) * cos(theta);
  const real_t R31 = -sin(theta);

  const real_t R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const real_t R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const real_t R32 = cos(theta) * sin(phi);

  const real_t R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  const real_t R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  const real_t R33 = cos(theta) * cos(phi);

  mat3_t R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

quat_t euler2quat(const vec3_t &euler) {
  const real_t phi = euler(1);
  const real_t theta = euler(2);
  const real_t psi = euler(3);

  const real_t c_phi = cos(phi / 2.0);
  const real_t c_theta = cos(theta / 2.0);
  const real_t c_psi = cos(psi / 2.0);
  const real_t s_phi = sin(phi / 2.0);
  const real_t s_theta = sin(theta / 2.0);
  const real_t s_psi = sin(psi / 2.0);

  const real_t qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  const real_t qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  const real_t qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  const real_t qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;

  const real_t mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  return quat_t{qw / mag, qx / mag, qy / mag, qz / mag};
}

mat3_t vecs2rot(const vec3_t &a_B, const vec3_t &g) {
  // Create Quaternion from two vectors
  const real_t cos_theta = a_B.normalized().transpose() * g.normalized();
  const real_t half_cos = sqrt(0.5 * (1.0 + cos_theta));
  const real_t half_sin = sqrt(0.5 * (1.0 - cos_theta));
  const vec3_t w = a_B.cross(g).normalized();

  const real_t qw = half_cos;
  const real_t qx = half_sin * w(0);
  const real_t qy = half_sin * w(1);
  const real_t qz = half_sin * w(2);

  // Convert Quaternion to rotation matrix
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  const real_t R11 = qw2 + qx2 - qy2 - qz2;
  const real_t R12 = 2 * (qx * qy - qw * qz);
  const real_t R13 = 2 * (qx * qz + qw * qy);

  const real_t R21 = 2 * (qx * qy + qw * qz);
  const real_t R22 = qw2 - qx2 + qy2 - qz2;
  const real_t R23 = 2 * (qy * qz - qw * qx);

  const real_t R31 = 2 * (qx * qz - qw * qy);
  const real_t R32 = 2 * (qy * qz + qw * qx);
  const real_t R33 = qw2 - qx2 - qy2 + qz2;

  mat3_t R;
  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
  return R;
}

mat3_t rvec2rot(const vec3_t &rvec, const real_t eps) {
  // Magnitude of rvec
  const real_t theta = sqrt(rvec.transpose() * rvec);
  // ^ basically norm(rvec), but faster

  // Check if rotation is too small
  if (theta < eps) {
    // clang-format off
    mat3_t R;
    R << 1, -rvec(2), rvec(1),
         rvec(2), 1, -rvec(0),
         -rvec(1), rvec(0), 1;
    return R;
    // clang-format on
  }

  // Convert rvec to rotation matrix
  const vec3_t rvec_normalized = rvec / theta;
  const real_t x = rvec_normalized(0);
  const real_t y = rvec_normalized(1);
  const real_t z = rvec_normalized(2);

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

  // clang-format off
  mat3_t R;
  R << x * xC + c, xyC - zs, zxC + ys,
       xyC + zs, y * yC + c, yzC - xs,
       zxC - ys, yzC + xs, z * zC + c;
  return R;
  // clang-format on
}

vec3_t quat2euler(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();

  const real_t qw2 = qw * qw;
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;

  const real_t t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const real_t t2 = asin(2 * (qy * qw - qx * qz));
  const real_t t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  return vec3_t{t1, t2, t3};
}

mat3_t quat2rot(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();

  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  // Homogeneous form
  mat3_t C;
  // -- 1st row
  C(0, 0) = qw2 + qx2 - qy2 - qz2;
  C(0, 1) = 2 * (qx * qy - qw * qz);
  C(0, 2) = 2 * (qx * qz + qw * qy);
  // -- 2nd row
  C(1, 3) = 2 * (qx * qy + qw * qz);
  C(1, 4) = qw2 - qx2 + qy2 - qz2;
  C(1, 5) = 2 * (qy * qz - qw * qx);
  // -- 3rd row
  C(2, 6) = 2 * (qx * qz - qw * qy);
  C(2, 7) = 2 * (qy * qz + qw * qx);
  C(2, 8) = qw2 - qx2 - qy2 + qz2;

  return C;
}

quat_t quat_delta(const vec3_t &dalpha) {
  const real_t half_norm = 0.5 * dalpha.norm();
  const vec3_t vector = sinc(half_norm) * 0.5 * dalpha;
  const real_t scalar = cos(half_norm);
  return quat_t{scalar, vector(0), vector(1), vector(2)};
}

mat4_t quat_lmul(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();
  mat4_t lmul;
  // clang-format off
  lmul << qw, -qx, -qy, -qz,
          qx,  qw, -qz,  qy,
          qy,  qz,  qw, -qx,
          qz, -qy,  qx,  qw;
  // clang-format on
  return lmul;
}

mat3_t quat_lmul_xyz(const quat_t &q) {
  mat4_t Q = quat_lmul(q);
  return Q.bottomRightCorner<3, 3>();
}

mat4_t quat_rmul(const quat_t &q) {
  const real_t qw = q.w();
  const real_t qx = q.x();
  const real_t qy = q.y();
  const real_t qz = q.z();
  mat4_t lmul;
  // clang-format off
  lmul << qw, -qx, -qy, -qz,
          qx,  qw,  qz, -qy,
          qy, -qz,  qw,  qx,
          qz,  qy, -qx,  qw;
  // clang-format on
  return lmul;
}

mat3_t quat_rmul_xyz(const quat_t &q) {
  mat4_t Q = quat_rmul(q);
  return Q.bottomRightCorner<3, 3>();
}

mat3_t quat_mat_xyz(const mat4_t &Q) {
  return Q.bottomRightCorner<3, 3>();
}

void imu_init_attitude(const vec3s_t w_m,
                       const vec3s_t a_m,
                       mat3_t &C_WS,
                       const size_t buffer_size) {
  // Sample IMU measurements
  vec3_t sum_angular_vel = vec3_t::Zero();
  vec3_t sum_linear_acc = vec3_t::Zero();
  for (size_t i = 0; i < buffer_size; i++) {
    sum_angular_vel += w_m[i];
    sum_linear_acc += a_m[i];
  }

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  const vec3_t mean_accel = sum_linear_acc / buffer_size;
  const vec3_t gravity{0.0, 0.0, -9.81};
  C_WS = vecs2rot(mean_accel, -gravity);
}

/*****************************************************************************
 *                                TIME
 *****************************************************************************/

void timestamp_print(const timestamp_t &ts, const std::string &prefix) {
  if (prefix != "") {
    printf("%s: " "%" PRIu64 "\n", prefix.c_str(), ts);
  } else {
    printf("%" PRIu64 "\n", ts);
  }
}

real_t ts2sec(const timestamp_t &ts) { return ts * 1.0e-9; }

real_t ns2sec(const uint64_t ns) { return ns * 1.0e-9; }

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

real_t time_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((real_t) t.tv_sec + ((real_t) t.tv_usec) / 1000000.0);
}

/*****************************************************************************
 *                                BA DATA
 ****************************************************************************/

pose_t::pose_t() {}

pose_t::pose_t(const mat4_t &T) {
  const quat_t q{tf_quat(T)};
  const vec3_t r{tf_trans(T)};

  param(0) = q.w();
  param(1) = q.x();
  param(2) = q.y();
  param(3) = q.z();

  param(4) = r(0);
  param(5) = r(1);
  param(6) = r(2);
}

quat_t pose_t::rot() const {
  return quat_t{param[0], param[1], param[2], param[3]};
}

vec3_t pose_t::trans() const {
  return vec3_t{param[4], param[5], param[6]};
}

mat4_t pose_t::T() const {
  return tf(rot(), trans());
}

quat_t pose_t::rot() { return static_cast<const pose_t &>(*this).rot(); }
vec3_t pose_t::trans() { return static_cast<const pose_t &>(*this).trans(); }
mat4_t pose_t::T() { return static_cast<const pose_t &>(*this).T(); }

void pose_t::set_trans(const vec3_t &r) {
  param(4) = r(0);
  param(5) = r(1);
  param(6) = r(2);
}

void pose_t::set_rot(const quat_t &q) {
  param(0) = q.w();
  param(1) = q.x();
  param(2) = q.y();
  param(3) = q.z();
}

void pose_t::set_rot(const mat3_t &C) {
  quat_t q{C};
  param(0) = q.w();
  param(1) = q.x();
  param(2) = q.y();
  param(3) = q.z();
}

void pose_print(const std::string &prefix, const pose_t &pose) {
  const quat_t q = pose.rot();
  const vec3_t r = pose.trans();

  printf("[%s] ", prefix.c_str());
  printf("q: (%f, %f, %f, %f)", q.w(), q.x(), q.y(), q.z());
  printf("\t");
  printf("r: (%f, %f, %f)\n", r(0), r(1), r(2));
}

poses_t load_poses(const std::string &csv_path) {
  FILE *csv_file = fopen(csv_path.c_str(), "r");
  char line[1024] = {0};
  poses_t poses;

  size_t pose_index = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[1024] = {0};
    real_t data[7] = {0};
    int index = 0;
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[index] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        index++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    quat_t q{data[0], data[1], data[2], data[3]};
    vec3_t r{data[4], data[5], data[6]};
    poses.emplace_back(tf(q, r));
    pose_index++;
  }
  fclose(csv_file);

  return poses;
}

keypoints_t parse_keypoints_line(const char *line) {
  char entry[100] = {0};
  int kp_ready = 0;
  vec2_t kp{0.0, 0.0};
  int kp_index = 0;
  bool first_element_parsed = false;

  // Parse line
  keypoints_t keypoints;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (first_element_parsed == false) {
        first_element_parsed = true;
      } else {
        // Parse keypoint
        if (kp_ready == 0) {
          kp(0) = strtod(entry, NULL);
          kp_ready = 1;

        } else {
          kp(1) = strtod(entry, NULL);
          keypoints.push_back(kp);
          kp_ready = 0;
          kp_index++;
        }
      }

      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return keypoints;
}

std::vector<keypoints_t> load_keypoints(const std::string &data_path) {
  char keypoints_csv[1000] = {0};
  strcat(keypoints_csv, data_path.c_str());
  strcat(keypoints_csv, "/keypoints.csv");

  FILE *csv_file = fopen(keypoints_csv, "r");
  std::vector<keypoints_t> keypoints;

  char line[1024] = {0};
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }
    keypoints.push_back(parse_keypoints_line(line));
  }
  fclose(csv_file);

  return keypoints;
}

void keypoints_print(const keypoints_t &keypoints) {
  printf("nb_keypoints: %zu\n", keypoints.size());
  printf("keypoints:\n");
  for (size_t i = 0; i < keypoints.size(); i++) {
    printf("-- (%f, %f)\n", keypoints[i](0), keypoints[i](1));
  }
}

mat3_t load_camera(const std::string &data_path) {
  // Setup csv path
  char cam_csv[1000] = {0};
  strcat(cam_csv, data_path.c_str());
  strcat(cam_csv, "/camera.csv");

  // Parse csv file
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **cam_K = csv_data(cam_csv, &nb_rows, &nb_cols);
  if (cam_K == NULL) {
    FATAL("Failed to load csv file [%s]!", cam_csv);
  }
  if (nb_rows != 3 || nb_cols != 3) {
    LOG_ERROR("Error while parsing camera file [%s]!", cam_csv);
    LOG_ERROR("-- Expected 3 rows got %d instead!", nb_rows);
    LOG_ERROR("-- Expected 3 cols got %d instead!", nb_cols);
    FATAL("Invalid camera file [%s]!", cam_csv);
  }

  // Flatten 2D array to 1D array
  mat3_t K;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      K(i, j) = cam_K[i][j];
    }
    free(cam_K[i]);
  }
  free(cam_K);

  return K;
}

poses_t load_camera_poses(const std::string &data_path) {
  char cam_poses_csv[1000] = {0};
  strcat(cam_poses_csv, data_path.c_str());
  strcat(cam_poses_csv, "/camera_poses.csv");
  return load_poses(cam_poses_csv);
}

poses_t load_target_pose(const std::string &data_path) {
  char target_pose_csv[1000] = {0};
  strcat(target_pose_csv, data_path.c_str());
  strcat(target_pose_csv, "/target_pose.csv");
  return load_poses(target_pose_csv);
}

real_t **load_points(const std::string &data_path, int *nb_points) {
  char points_csv[1000] = {0};
  strcat(points_csv, data_path.c_str());
  strcat(points_csv, "/points.csv");

  // Initialize memory for points
  *nb_points = csv_rows(points_csv);
  real_t **points = (real_t **) malloc(sizeof(real_t *) * *nb_points);
  for (int i = 0; i < *nb_points; i++) {
    points[i] = (real_t *) malloc(sizeof(real_t) * 3);
  }

  // Load file
  FILE *infile = fopen(points_csv, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Loop through data
  char line[1024] = {0};
  size_t len_max = 1024;
  int point_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        points[point_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    point_idx++;
  }

  // Cleanup
  fclose(infile);

  return points;
}

int **load_point_ids(const std::string &data_path, int *nb_points) {
  char csv_path[1000] = {0};
  strcat(csv_path, data_path.c_str());
  strcat(csv_path, "/point_ids.csv");
  return load_iarrays(csv_path, nb_points);
}
