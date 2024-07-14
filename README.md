# IMPACTOR: <u>IMP</u>act-<u>A</u>ware Planning and <u>C</u>on<u>T</u>r<u>O</u>l for Aerial <u>R</u>obots with Suspended Payloads
## News
- 26 Mar., 2024: Released the impact-aware planning algorithm and the early access version paper.
- 15 Jul., 2024: Released the simulation and controller code and the official version paper.
## TODO
- [x] Release impact-aware planning algorithms.
- [x] Release simulation code. 
- [x] Release hybrid MPC code.
- [ ] Update user guide.
- [ ] Release Docker image.
## Content
* [Introduction](#introduction)
## Introduction
<div align=center>
  <img src="images/gif-scenario_1.gif" width=400px>
  <img src="images/gif-scenario_2.gif" width=400px>
</div>

This repository contains the source code of the impact-aware planning and control algorithms described in our paper "Impact-Aware Planning and Control for Aerial Robots with Suspended Payloads." accepted by _IEEE Transactions on Robotics (T-RO)_, 2024.

__Authors__: [Haokun Wang](https://haokun-wang.com)<sup>1+</sup>, Haojia Li<sup>1+</sup>, [Boyu Zhou](https://boyuzhou.net/)<sup>2*</sup>, [Fei Gao](http://zju-fast.com/fei-gao/)<sup>3*</sup> and [Shaojie Shen](https://uav.hkust.edu.hk/group/)<sup>1</sup>

<small><sup>1</sup>[HKUST Aerial Robotics Group](https://uav.hkust.edu.hk/), <sup>2</sup> [SYSU STAR Lab](https://boyuzhou.net/), <sup>3</sup> [ZJU FAST Lab](http://zju-fast.com/), .</small>

__Paper__: arXiv, [IEEE Official Version](https://ieeexplore.ieee.org/abstract/document/10478625)

__Supplementary Video__: [YouTube](https://youtu.be/k_XGQyrNh9I?si=K2775t8ui0WClqqv), [Bilibili](https://www.bilibili.com/video/BV1zg4y1L7dC/?share_source=copy_web&vd_source=4a496bdfc1980dd80977a281d5c963c0)

__Project Website__: [Homepage](https://sites.google.com/view/suspended-payload/)

_Abstract_: A quadrotor with a cable-suspended payload imposes great challenges in impact-aware planning and control. This joint system has dual motion modes, depending on whether the cable is slack or not, and presents complicated dynamics. Therefore, generating feasible agile flight while preserving the retractable nature of the cable is still a challenging task. In this paper, we propose a novel impact-aware planning and control framework that resolves potential impacts caused by motion mode switching. Our method leverages the augmented Lagrangian method (ALM) to solve an optimization problem with nonlinear complementarity constraints (ONCC), which ensures trajectory feasibility with high accuracy while maintaining efficiency. We further propose a hybrid nonlinear model predictive control method to address the model mismatch issue in agile flight. Our methods have been comprehensively validated in both simulation and experiments, demonstrating superior performance compared to existing approaches. To the best of our knowledge, we are the first to successfully perform automatic multiple motion mode switching for aerial payload systems in real-world experiments.

![SystemDiagram](images/fig-system_diagram.png)

## Demonstrations
- Visualization using RViz.
<div align=center>
  <img src="images/gif-benchmark_0.gif" width=200px>
  <img src="images/gif-benchmark_1.gif" width=200px>
  <img src="images/gif-benchmark_2.gif" width=200px>
  <img src="images/gif-benchmark_3.gif" width=200px>
</div>
<div align=center>
  <img src="images/gif-benchmark_4.gif" width=200px>
  <img src="images/gif-benchmark_5.gif" width=200px>
  <img src="images/gif-benchmark_6.gif" width=200px>
  <img src="images/gif-benchmark_7.gif" width=200px>
</div>

- Simulations using Drake.
<div align=center>
  <img src="images/gif-sim_scenario_1.gif" width=400px>
  <img src="images/gif-sim_scenario_2.gif" width=400px>
</div>