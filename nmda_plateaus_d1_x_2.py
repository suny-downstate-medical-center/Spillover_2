# -*- coding: utf-8 -*-
"""
Created on Fri Aug  2 17:50:36 2019

@author: daniel
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 17:48:41 2016

@author: daniel
"""
from neuron import h,rxd
import d1msn_2 as msn
#import iMSN
import spillover_experiment as pe
import pickle
import parameters as p
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import json
import scipy.signal as ss
import os, sys, traceback

# --- 1. Create a cell and other useful stuff
dMSN_library = 'D1_71bestFit_updRheob.pkl'
iMSN_library = 'D2_34bestFit_updRheob.pkl' 
with open(dMSN_library, 'rb') as f:
    model_sets  = pickle.load(f, encoding="latin1")

cell_ID = 58

cell_ids = list(model_sets.keys())

for cell_id in cell_ids:
    print (" cell id ", cell_id)
    variables = model_sets[cell_id]['variables'] 
    # print(variables)
    cell = msn.MSN(variables = variables) 
    for d in p.input_dends:
        print (" input dendrite ", d)
        cell.dendlist[d].nseg *=5
        dend_record_list = [22] #[3,4,9,10,21,22,24,26,35,36,51,52]
        dend_stim_list = []#[3,4,9,10,35,36]                    
        plateau_cluster_list = [22]        

        plateau_cluster_size = np.arange(1,31,1)

        vs = []
        vspine = []
        vd = []
        legend = []

        max_vs = []
        max_vspine = []
        max_vd = []
        g_nmda = []
        i_nmda = []

        sns.set(font_scale = 1.0)
        sns.set_style('whitegrid')
        fig_vs = plt.figure(); 
        fig_vspine = plt.figure(); 
        fig_vd = plt.figure();
        
        ax_vs = fig_vs.add_subplot(111); ax_vs.set_ylabel('Vs (mV)'); ax_vs.set_xlabel('t (ms)')
        ax_vspine = fig_vspine.add_subplot(111); ax_vspine.set_ylabel('Vspine (mV)'); ax_vspine.set_xlabel('t (ms)')
        ax_vd = fig_vd.add_subplot(111); ax_vd.set_ylabel('Vd (mV)'); ax_vd.set_xlabel('t (ms)')
        colors = sns.color_palette("coolwarm", plateau_cluster_size.max())

        add_spine = 0
        on_spine = 1

        cell.insert_spines(plateau_cluster_list, p.cluster_start_pos, p.cluster_end_pos, num_spines = p.plateau_cluster_size_max)             
        sns.set_style("ticks")

    for index_plateau, num_syns in enumerate(plateau_cluster_size):
        print (" index_plateau ", index_plateau)

        ex = pe.Spillover_Experiment('record_ca', cell)
    #     ex.insert_synapses('noise_SPN')
    #     try:
    #         ex.insert_synapses('my_spillover', plateau_cluster_list, deterministic = 0, 
    #                     num_syns = num_syns, add_spine = add_spine, on_spine = on_spine)
    #     except:
    #         traceback.print_exc(file=sys.stdout)
