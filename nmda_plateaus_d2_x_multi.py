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
# import d1msn as msn
import iMSN as msn
import spillover_experiment as pe
import pickle
import parameters_2 as p
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import json
import scipy.signal as ss
import os, sys, traceback
from multiprocessing import Pool, TimeoutError
# --- 1. Create a cell and other useful stuff
dMSN_library = 'D1_71bestFit_updRheob.pkl'
iMSN_library = 'D2_34bestFit_updRheob.pkl' 
with open(iMSN_library, 'rb') as f:
    model_sets  = pickle.load(f, encoding="latin1")

cell_ID = 58

cell_ids = list(model_sets.keys())

# pool = Pool(len(cell_ids))

# def run_cell_sim(cell_id):
for cell_id in cell_ids:
    print (" cell id ", cell_id)
    if cell_id <2:
        continue

    variables = model_sets[cell_id]['variables'] 
    print(variables)
    cell = msn.iMSN(variables = variables) 
    for d in p.input_dends:
        print (" input dendrite ", d)
        for x in cell.dendlist:
            print (x)
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
        ex.insert_synapses('noise_SPN')
        ex.insert_synapses('my_spillover', plateau_cluster_list, deterministic = 0, 
                        num_syns = num_syns, add_spine = add_spine, on_spine = on_spine)

        ex.set_up_recording(dend_record_list)
        ex.simulate()
        tv = ex.tv.to_python()
        t = ex.t.to_python()
        legend.append("%d syns" % num_syns)

        if add_spine == 1 or on_spine == 1:
            vspine.append(ex.vspine[0].to_python())
            max_vspine.append(max(ex.vspine[0]))
            ax_vspine.plot(tv, ex.vspine[0].to_python(), color = colors[num_syns-1])

        vd.append(ex.vdlist[0].to_python())
        max_vd.append(max(ex.vdlist[0])) 
        
        max_vs.append(max(ex.vs))
        vs.append(ex.vs.to_python())
            
        cell.esyn = []
        ex.estim = []
        ex.enc = []
        for s in cell.spines:
            s.syn_on = 0
        cell.isyn = []
        ex.istim = []
        ex.inc = []

    vs_indices = []; vs_widths = []
    vd_indices = []; vd_widths = []
    vspine_indices = []; vspine_wid = []
    for v in vs:
        vs_indices.append(ss.find_peaks(v))
        vs_widths.append(ss.peak_widths(v, (vs_indices[-1])[0], rel_height = 0.15))

    for v in vd:
        vd_indices.append(ss.find_peaks(v))
        vd_widths.append(ss.peak_widths(v, (vd_indices[-1])[0] ,rel_height = 0.15))
        
    for i in range(0, len(vs)):    
        if add_spine ==0 and on_spine ==0:    
            ax_vd.plot(tv, vd[i]);   
        ax_vs.plot(tv, vs[i], color = colors[i]); ax_vs.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
        ax_vd.plot(tv, vd[i], color = colors[i]); ax_vs.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
    ax_vd.set_title("cell_id %d weight = %.2f, Cdur_factor = %d" % (cell_id, p.weight, p.eCdur_factor))
        
    sns.despine()
    res_dict = {'t': t,
                'vs': vs,
                'vspine': vspine}
    to_save = json.dumps(res_dict)
    #filename = './results/data_spillover_steep.dat'
    #with open(filename,'w', encoding = 'utf-8') as f:
    #    json.dump(to_save, f)

    # plt.show()

    plt.savefig("d2_results_" + str(cell_id) + ".png")
    # except:
    #     traceback.print_exc(file=sys.stdout)
# funclist = []
# for cell_id in cell_ids:
#     print (" cell id ", cell_id)
#     # if cell_id == 1:
#     #     continue
#     f = pool.map(run_cell_sim,(cell_id))
#     funclist.append(f)
# results = []
# for f in funclist:
#     try:
#         results.append(f.get())
#     except:
#         pass