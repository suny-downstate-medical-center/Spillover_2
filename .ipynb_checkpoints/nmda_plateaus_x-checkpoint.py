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

from neuron import h
import d1msn
import d2msn
import spillover_experiment as pe
import pickle
import parameters as p
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import json
import scipy.signal as ss
import pandas as pd
import sys

# --- 1. Create a cell and other useful stuff
dMSN_library = 'D1_71bestFit_updRheob.pkl'
iMSN_library = 'D2_34bestFit_updRheob.pkl' 

class nmda_sim:

    def __init__(self, neuron_id,neuron_name,archive,length,cell_type_val):

        dMSN_library = 'D1_71bestFit_updRheob.pkl'
        iMSN_library = 'D2_34bestFit_updRheob.pkl' 

        self.data_list = []
        self.neuron_id_list = []

        self.neuron_id = neuron_id
        self.neuron_name = neuron_name
        self.archive = archive
        self.sec_num = 0
        self.variables = ''
        self.params = ''
        # self.morphology_file = morphology_file
        self.cell_type_val = cell_type_val

        self.error = False
        # self.insert_channels(variables)
        self.esyn = []
        self.isyn = []
        self.spines = []
        # self.num_spines_on_dends = np.zeros(len(self.dendlist))
        
        found = False
        print ( " ********* ")

        files=[]
        
        d1_files = []
        d2_files = []
        
        d1_files_good = []
        d2_files_good = []
        
        d1_files_bad = []
        d2_files_bad = []
        
        cell_ID =22
        with open(dMSN_library, 'rb') as f:
            d1_model_sets  = pickle.load(f, encoding="latin1")
            self.variables = d1_model_sets[cell_ID]['variables']        
        
        with open(iMSN_library, 'rb') as f:
            d2_model_sets  = pickle.load(f, encoding="latin1")
            self.variables = d2_model_sets[cell_ID]['variables']        
            
        morphology = '../all_morphologies/' + self.archive.lower() + '/CNG version/' + self.neuron_name + '.CNG.swc'
        print ('../all_morphologies/' + self.archive.lower() + '/CNG version/' + self.neuron_name + '.CNG.swc')

        if cell_type_val == 'D1':
            cell = d1msn.MSN(morphology=morphology, variables = self.variables) 
        elif cell_type_val == 'D2':
            cell = d2msn.MSN(morphology=morphology, variables = self.variables) 

        # for d in p.input_dends:
        #     cell.dendlist[d].nseg *=5
        d_counter = 0
        
        for d in cell.dendlist:
        
            print (" counter ", d_counter)
            try:
                cell.dendlist[d_counter].nseg *= 5
            except:
                pass
            d_counter += 1
        
        print(cell.max_dist())
        dmax, sec = cell.max_dist()
        sec_name = sec.name()
        sec_index = sec_name.find("[")
        sec_num = int(sec_name[sec_index+1:sec_name.find("]")])
        print ( " sec num ", sec_num ) 

        dend_record_list = [sec_num] #[3,4,9,10,21,22,24,26,35,36,51,52]
        dend_stim_list = []#[3,4,9,10,35,36]                    
        plateau_cluster_list = [sec_num]        

        plateau_cluster_size = np.arange(1,31,1)
        # plateau_cluster_size = np.arange(1,3,1)
        
        vs = []
        vspine = []
        vd = []
        legend = []
        
        max_vs = []
        max_vspine = []
        max_vd = []
        g_nmda = []
        i_nmda = []

        plt.clf()
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
        for num_syns in plateau_cluster_size:
            ex = pe.Spillover_Experiment('record_ca', cell,input_dends=[sec_num],cluster_start_pos = [0.3], cluster_end_pos = [0.45])
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
        #
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
        ax_vd.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
            
        sns.despine()
        res_dict = {'t': t,
                    'vs': vs,
                    'vspine': vspine}
        to_save = json.dumps(res_dict)

        try:

            res_dict_vs = {'vs_indices': list(vs_indices),
                        'vs_widths': list(vs_widths)
                          }
            to_save_vs = json.dumps(res_dict_vs)
            filename = "plots/" + cell_type_val + "/data/" + archive.lower() + "_" + neuron_name + "_vs.dat"
            with open(filename,'w', encoding = 'utf-8') as f:
               json.dump(to_save_vs, f)
        except:
            pass
        try:
                
            res_dict_vd = {'vd_indices': list(vd_indices),
                        'vd_widths': list(vd_widths)
                          }                
            to_save_vd = json.dumps(res_dict_vd)


            filename = "plots/" + cell_type_val + "/data/" + archive.lower() + "_" + neuron_name + "_vd.dat"
            with open(filename,'w', encoding = 'utf-8') as f:
               json.dump(to_save_vd, f)
        except:
            pass
            
        plt.savefig("plots/" + cell_type_val + "/" + archive.lower() + "_" + neuron_name + ".png")

if __name__ == '__main__':

    neuron_id = sys.argv[1]
    neuron_name = sys.argv[2]
    archive = sys.argv[3]
    length = float(sys.argv[4])
    cell_type_val = sys.argv[5]

    sim = nmda_sim(neuron_id,neuron_name,archive,length,cell_type_val)