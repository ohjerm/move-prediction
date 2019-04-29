#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 11:40:12 2019

@author: oliver
"""

import numpy as np
import matplotlib.pyplot as plt

t_1 = [np.NaN, 32.25, np.NaN, 33.84, 29.88, 30.69, 29.57, 29.75, 30.79, 31.48]
x_lab = range(1,11)

idx = np.isfinite(x_lab) & np.isfinite(t_1)

fit = np.polyfit(x_lab[idx], t_1[idx], 1)
fit_fn = np.poly1d(fit)

plt.plot(x_lab, t_1, 'o', x_lab, fit_fn(x_lab), '-')
plt.ylim([0, 50])