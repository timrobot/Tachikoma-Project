Tensor &Tensor::operator+=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_addI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator-=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_subI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator*=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_mulI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator/=(const float &f) {
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_divI<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator+=(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_add<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator-=(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_sub<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator%=(const Tensor &other) { // schur product
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_mul<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator/=(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_div<<<gridSize, blockSize>>>(this->d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return *this;
}

Tensor &Tensor::operator*=(const Tensor &other) {
  Tensor G = (*this) * other;
  this->destroy();
  this->d_pixels = G.d_pixels;
  this->n_rows = G.n_rows;
  this->n_cols = G.n_cols;
  this->n_slices = G.n_slices;
  this->n_elem = G.n_elem;
  return *this;
}

Tensor Tensor::operator+(const float &f) {
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_addI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator-(const float &f) {
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_subI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator*(const float &f) {
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_mulI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator/(const float &f) {
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_divI<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, f, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator+(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_add<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator-(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1) / 16 + 1, (this->n_cols-1) / 16 + 1, 1);
  GPU_sub<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator%(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1)/16+1, (this->n_cols-1)/16+1, 1);
  GPU_mul<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator/(const Tensor &other) {
  assert(this->n_rows == other.n_rows && this->n_cols == other.n_cols);
  Tensor G(this->n_rows, this->n_cols, 1, gfill::none);
  dim3 blockSize(16, 16, 1);
  dim3 gridSize((this->n_rows-1)/16+1, (this->n_cols-1)/16 + 1, 1);
  GPU_div<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}

Tensor Tensor::operator*(const Tensor &other) {
  assert(this->n_cols == other.n_rows);
  Tensor G(this->n_cols, this->n_rows * other.n_cols, 1, gfill::none);
  dim3 blockSize(8, 8, 8);
  dim3 gridSize((this->n_rows-1)/8+1, (other.n_cols-1)/8+1, (this->n_cols-1)/8+1);
  // set up the matrices (map mult)
  GPU_mmul<<<gridSize, blockSize>>>(G.d_pixels, this->d_pixels, other.d_pixels, this->n_rows, other.n_cols, this->n_cols);
  checkCudaErrors(cudaGetLastError());
  // sum up each column
  blockSize.x = 128;
  blockSize.y = 1;
  blockSize.z = 1;
  gridSize.x = (G.n_rows-1)/128+1;
  gridSize.y = G.n_cols;
  gridSize.z = 1;
  for (int i = 0; (size_t)(1 << i) < G.n_rows; i += 8) {
    GPU_sum<<<gridSize, blockSize, sizeof(float) * 128>>>(G.d_pixels, G.d_pixels, G.n_rows, G.n_cols, 128, i);
    checkCudaErrors(cudaGetLastError());
    blockSize.x = MIN(gridSize.x, 128);
    gridSize.x = (blockSize.x-1)/128+1;
  }
  blockSize.x = 128;
  gridSize.x = (G.n_rows-1)/128+1;
  gridSize.y = 1;
  Tensor F(this->n_rows * other.n_cols, 1, 1, gfill::none);
  GPU_copyRow<<<gridSize, blockSize>>>(F.d_pixels, G.d_pixels, F.n_rows, 0);
  checkCudaErrors(cudaGetLastError());
  return F;
}
