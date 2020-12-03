## 1. Overview

This package is a C++ implementation of essential matrix estimation, which is called [N-point method](https://arxiv.org/pdf/1903.09067v3.pdf) [1]. 

**Authors:** [Ji Zhao](https://sites.google.com/site/drjizhao)

If you use this code for your research, please cite:
```
@article{zhao2020efficient,
  title={An Efficient Solution to Non-Minimal Case Essential Matrix Estimation},
  author={Zhao, Ji},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence},
  year={2020},
  doi = {10.1109/TPAMI.2020.3030161},
  publisher={IEEE}
}
```

## 2. Quick Start

### 2.1 Dependencies

1) Eigen 3. http://eigen.tuxfamily.org

2) SDPA 7.3.8. http://sdpa.sourceforge.net/

### 2.2 How to Build 

```sh
git clone git@github.com:jizhaox/npt-pose.git
cd npt-pose
mkdir build && cd build
cmake ..
make
./demo
```


