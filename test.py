# -*- coding: utf-8 -*-
"""
Created on Fri Aug  2 17:50:36 2019

@author: daniel
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 17:48:41 2016

@author: sid
"""
import os,sys,traceback
from neuron import h
import d1msn as d1msn
import d2msn as d2msn
import spillover_experiment as pe
import pickle
import parameters as p
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import json
import scipy.signal as ss
import pandas as pd
import spine as sp
from math import exp

class nmda_sim:

    def __init__(self):

        dMSN_library = 'D1_71bestFit_updRheob.pkl'
        iMSN_library = 'D2_34bestFit_updRheob.pkl' 

        self.data_list = []
        self.neuron_id_list = []

        self.neuron_id = ''
        self.neuron_name = ''
        self.archive = ''
        self.note = ''
        self.sec_num = 0
        self.variables = ''
        self.params = ''

        self.error = False
        # self.insert_channels(variables)
        self.esyn = []
        self.isyn = []
        self.spines = []
        # self.num_spines_on_dends = np.zeros(len(self.dendlist))
        
        found = False
        print ( " ********* ")
        # for dir_name in os.listdir('morphology/msn_morphologies'):
        #     data = dir_name.split("_")
            
        #     folder_number = int(data[1])
        files=[]
        
        d1_files = []
        d2_files = []
        
        d1_files_good = []
        d2_files_good = []
        
        d1_files_bad = []
        d2_files_bad = []
        
        df = pd.read_csv('soma.tsv',index_col=None, sep="\t")
        print(df.columns)
        df.head()
        cell_ID =22
        with open(dMSN_library, 'rb') as f:
            d1_model_sets  = pickle.load(f, encoding="latin1")
            self.variables = d1_model_sets[cell_ID]['variables']        
        
        with open(iMSN_library, 'rb') as f:
            d2_model_sets  = pickle.load(f, encoding="latin1")
            self.variables = d2_model_sets[cell_ID]['variables']        
        
        for index,row in df.iterrows():

            print (row['domain'])

            self.neuron_id = row['neuron_id']
            self.neuron_name = row['neuron_name']
            self.archive = row['archive']
            self.note = row['note']
            self.length = row['length']
            cell_type_val = row['cell_type_val']

            if self.length < 4000: 
                continue
                
            archive = row['archive']
            neuron_name = row['neuron_name']
            
            if cell_type_val and isinstance(cell_type_val, str):
                file_name = '/u/sid/all_morphologies/' + archive.lower() + '/CNG version/' + neuron_name + '.CNG.swc'
                if cell_type_val == 'D1':

                    print (" $$ D1 $$ ")
                    
                    d1_files.append(file_name)  
                    self.params = "./params_dMSN.json"
        
                    try:
                        Import = h.Import3d_SWC_read()
                        Import.input(file_name)
                        imprt = h.Import3d_GUI(Import, 0)
                        imprt.instantiate(None)
                        h.define_shape()         

                        self.create_sectionlists()
                        self.set_nsegs()
                        
                        d1_files_good.append(file_name)  
                        h.celsius = 35
                        self.v_init = -80

                        dmax, sec = self.max_dist()
                        print('dmax ', dmax, ' sec ' , sec)
                        
                        sec_name = sec.name()
                        sec_index = sec_name.find("[")
                        sec_num = int(sec_name[sec_index+1:sec_name.find("]")])
                        print ( " sec num ", sec_num )   

                        self.sec_num = sec_num

                        self.run_cell_sim()                    
                        
                    except:
                        traceback.print_exc(file=sys.stdout)
                        print (" error loading file " , file_name)
                        d1_files_bad.append(file_name)  
                        
                elif cell_type_val == 'D2':

                    print (" $$ D2 $$ ")
                    
                    d2_files.append(file_name)  
                    self.params = "./params_iMSN.json"
        
                    try:
                        Import = h.Import3d_SWC_read()
                        Import.input(file_name)
                        imprt = h.Import3d_GUI(Import, 0)
                        imprt.instantiate(None)
                        h.define_shape()         
                        self.create_sectionlists()
                        self.set_nsegs()
                        h.celsius = 35
                        self.v_init = -80
                        dmax, sec = self.max_dist()
                        print('dmax ', dmax, ' sec ' , sec)

                        d2_files_good.append(file_name)  
                        
                        sec_name = sec.name()
                        sec_index = sec_name.find("[")
                        sec_num = int(sec_name[sec_index+1:sec_name.find("]")])
                        print ( " sec num ", sec_num ) 
                        self.sec_num = sec_num
    
                        self.run_cell_sim()
        
                    except:
                        traceback.print_exc(file=sys.stdout)
                        print (" error loading file " , file_name)
                        d2_files_bad.append(file_name)                  
            # break
        
        df_good = pd.DataFrame(data = self.data_list, columns = ['neuron_id','neuron_name','archive','note'], index = None)
        
        df_good.to_csv("good_files.tsv", sep="\t", index=None)

        df_soma = df[df['neuron_id'].isin(self.neuron_id_list)]
        df_soma.to_csv("soma.tsv",sep="\t",index=None)
        
        print (" all d1 = " , len(d1_files))
        print (" d1 good " , len(d1_files_good))
        print (" d1 bad " , len(d1_files_bad))
        
        print (" all d2 ", len(d2_files))
        print (" d2 good ", len(d2_files_good))
        print (" d2 bad " , len(d2_files_bad))

    def run_cell_sim(self):
        try:

            self.insert_channels(self)
            
            dend_record_list = [self.sec_num] #[3,4,9,10,21,22,24,26,35,36,51,52]
            dend_stim_list = []#[3,4,9,10,35,36]                    
            plateau_cluster_list = [self.sec_num]        
    
            plateau_cluster_size = np.arange(1,5,1)
    
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
            # fig_vs = plt.figure(); 
            # fig_vspine = plt.figure(); 
            # fig_vd = plt.figure();
            
            # ax_vs = fig_vs.add_subplot(111); ax_vs.set_ylabel('Vs (mV)'); ax_vs.set_xlabel('t (ms)')
            # ax_vspine = fig_vspine.add_subplot(111); ax_vspine.set_ylabel('Vspine (mV)'); ax_vspine.set_xlabel('t (ms)')
            # ax_vd = fig_vd.add_subplot(111); ax_vd.set_ylabel('Vd (mV)'); ax_vd.set_xlabel('t (ms)')
            # colors = sns.color_palette("coolwarm", plateau_cluster_size.max())
    
            add_spine = 0
            on_spine = 1
    
            self.insert_spines(plateau_cluster_list, p.cluster_start_pos, p.cluster_end_pos, num_spines = p.plateau_cluster_size_max)             
    
            # sns.set_style("ticks")
            # for num_syns in plateau_cluster_size:
    
            #     ex = pe.Spillover_Experiment('record_ca', cell)
            #     ex.insert_synapses('noise_SPN')
            #     ex.insert_synapses('my_spillover', plateau_cluster_list, deterministic = 0, 
            #                     num_syns = num_syns, add_spine = add_spine, on_spine = on_spine)
    
            #     ex.set_up_recording(dend_record_list)
            #     ex.simulate()
            #     tv = ex.tv.to_python()
            #     t = ex.t.to_python()
            #     legend.append("%d syns" % num_syns)
    
            #     if add_spine == 1 or on_spine == 1:
            #         vspine.append(ex.vspine[0].to_python())
            #         max_vspine.append(max(ex.vspine[0]))
            #         ax_vspine.plot(tv, ex.vspine[0].to_python(), color = colors[num_syns-1])
    
            #     vd.append(ex.vdlist[0].to_python())
            #     max_vd.append(max(ex.vdlist[0])) 
                
            #     max_vs.append(max(ex.vs))
            #     vs.append(ex.vs.to_python())
                    
            #     cell.esyn = []
            #     ex.estim = []
            #     ex.enc = []
            #     for s in cell.spines:
            #         s.syn_on = 0
            #     cell.isyn = []
            #     ex.istim = []
            #     ex.inc = []
            # #
            # vs_indices = []; vs_widths = []
            # vd_indices = []; vd_widths = []
            # vspine_indices = []; vspine_wid = []
            # for v in vs:
            #     vs_indices.append(ss.find_peaks(v))
            #     vs_widths.append(ss.peak_widths(v, (vs_indices[-1])[0], rel_height = 0.15))
    
            # for v in vd:
            #     vd_indices.append(ss.find_peaks(v))
            #     vd_widths.append(ss.peak_widths(v, (vd_indices[-1])[0] ,rel_height = 0.15))
                
            # for i in range(0, len(vs)):    
            #     if add_spine ==0 and on_spine ==0:    
            #         ax_vd.plot(tv, vd[i]);   
            #     ax_vs.plot(tv, vs[i], color = colors[i]); ax_vs.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
            #     ax_vd.plot(tv, vd[i], color = colors[i]); ax_vs.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
            # ax_vd.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
                
            # sns.despine()
            # res_dict = {'t': t,
            #             'vs': vs,
            #             'vspine': vspine}
            # to_save = json.dumps(res_dict)
            # #filename = './results/data_spillover_steep.dat'
            # #with open(filename,'w', encoding = 'utf-8') as f:
            # #    json.dump(to_save, f)
            # plt.savefig("plots/" + cell_type + "_" + str(folder_number)+"_" + morphology.replace(".","_") + ".png")
    
        except:
            traceback.print_exc(file=sys.stdout)

                   
    def insert_channels(self, variables = None):
        self.dendritic_channels =   [
                    "naf",      
                    "kaf",
                    "kas",
                    "kdr",
                    "kir",
                    "cal12",
                    "cal13",
                    "can",
                    "car",
                    "cav32",
                    "cav33",
                    "sk",
                    "bk"            ]
                
        self.somatic_channels = [
                    "naf",
                    "kaf",
                    "kas",
                    "kdr",
                    "kir",
                    "cal12",
                    "cal13",
                    "can",
                    "car",
                    "sk",
                    "bk"        ]
                    
        self.axonal_channels = [
                    "naf",
                    "kas",
                    "Im"        ]        

        # Load ion channel parameters
        with open(self.params) as file:
            par = json.load(file)
        
        for sec in self.somalist:
            for mech in self.somatic_channels+["cadyn", "caldyn"]:
                sec.insert(mech)
                
        for sec in self.axonlist:
            for mech in self.axonal_channels:
                sec.insert(mech)
                
        for sec in self.dendlist:
            for mech in self.dendritic_channels+["cadyn", "caldyn", "cadyn_nmda"]:
                sec.insert(mech)
            sec.taur_cadyn_nmda = p.tau_cadyn_nmda

        for sec in self.all:
            sec.Ra = 150
            sec.cm = 1.0
            sec.insert('pas')
            sec.g_pas = float(par['g_pas_all']['Value'])
            sec.e_pas = -70 # -73
            sec.ena = 50
            sec.ek = -85 # -90
            # print (" after ek " ) 

        # print(" naf ")
        self.distribute_channels("soma", "gbar_naf",   0, 1, 0, 0, 0, float(par['gbar_naf_somatic']['Value']))
        # print (" after naf ")
        self.distribute_channels("soma", "gbar_kaf",   0, 1, 0, 0, 0, float(par['gbar_kaf_somatic']['Value']))
        self.distribute_channels("soma", "gbar_kas",   0, 1, 0, 0, 0, float(par['gbar_kas_somatic']['Value']))
        self.distribute_channels("soma", "gbar_kdr",   0, 1, 0, 0, 0, float(par['gbar_kdr_somatic']['Value']))
        self.distribute_channels("soma", "gbar_bk",    0, 1, 0, 0, 0, float(par['gbar_bk_somatic' ]['Value']))
        self.distribute_channels("soma", "pbar_cal12", 0, 1, 0, 0, 0, 1.34e-5)
        self.distribute_channels("soma", "pbar_cal13", 0, 1, 0, 0, 0, 1.34e-6)
        self.distribute_channels("soma", "pbar_car",   0, 1, 0, 0, 0, 1.34e-4)
        self.distribute_channels("soma", "pbar_can",   0, 1, 0, 0, 0,    4e-5)
        
        self.distribute_channels("dend", "gbar_kdr",   0, 1, 0, 0, 0, float(par['gbar_kdr_basal']['Value']))
        self.distribute_channels("dend", "gbar_bk",    0, 1, 0, 0, 0, float(par['gbar_bk_basal' ]['Value']))

        self.distribute_channels("dend", "pbar_cal12", 0, 1, 0, 0, 0, 1e-5)
        self.distribute_channels("dend", "pbar_cal13", 0, 1, 0, 0, 0, 1e-6)
        self.distribute_channels("dend", "pbar_car",   0, 1, 0, 0, 0, 1e-4)
        
        self.distribute_channels("axon", "gbar_kas",   0, 1, 0, 0, 0,      float(par['gbar_kas_axonal']['Value']))
        self.distribute_channels("axon", "gbar_naf",   3, 1, 1.1, 30, 500, float(par['gbar_naf_axonal']['Value']))
        #self.distribute_channels("axon", "gbar_naf",   1, 1, 0.1, 30, -1, float(par['gbar_naf_axonal']['Value']))
        self.distribute_channels("axon", "gImbar_Im",   0, 1, 0, 0, 0, 1.0e-3)

                
        if self.variables:
            # print ("variables naf ", variables['naf'], "par['gbar_naf_basal']" , par['gbar_naf_basal'])
            self.distribute_channels("dend", "gbar_naf", 1,   1.0-self.variables['naf'][1],  \
                                                              self.variables['naf'][1],      \
                                                              self.variables['naf'][2],      \
                                                              self.variables['naf'][3],      \
                                                              np.power(10,self.variables['naf'][0])*float(par['gbar_naf_basal']['Value']))
            self.distribute_channels("dend", "gbar_kaf", 1,   1.0,                      \
                                                              self.variables['kaf'][1],      \
                                                              self.variables['kaf'][2],      \
                                                              self.variables['kaf'][3],      \
                                                              np.power(10,self.variables['kaf'][0])*float(par['gbar_kaf_basal']['Value']))
            self.distribute_channels("dend", "gbar_kas", 1,   0.1,                      \
                                                              0.9,                      \
                                                              self.variables['kas'][1],      \
                                                              self.variables['kas'][2],      \
                                                              np.power(10,self.variables['kas'][0])*float(par['gbar_kas_basal']['Value']))
                                                              
            self.distribute_channels("dend", "gbar_kir", 0,   np.power(10,self.variables['kir'][0]), 0, 0, 0,    float(par['gbar_kir_basal'  ]['Value']))
            self.distribute_channels("soma", "gbar_kir", 0,   np.power(10,self.variables['kir'][0]), 0, 0, 0,    float(par['gbar_kir_somatic']['Value']))
            self.distribute_channels("dend", "gbar_sk",  0,   np.power(10,self.variables['sk' ][0]), 0, 0, 0,    float(par['gbar_sk_basal'   ]['Value']))
            self.distribute_channels("soma", "gbar_sk",  0,   np.power(10,self.variables['sk' ][0]), 0, 0, 0,    float(par['gbar_sk_somatic' ]['Value']))

            self.distribute_channels("dend", "pbar_can",   1, 1.0-self.variables['can'][1],  \
                                                              self.variables['can'][1],      \
                                                              self.variables['can'][2],      \
                                                              self.variables['can'][3],      \
                                                              np.power(10,self.variables['can'][0]))
            self.distribute_channels("dend", "pbar_cav32", 1, 0,                        \
                                                              1,                        \
                                                              self.variables['c32'][1],      \
                                                              self.variables['c32'][2],      \
                                                              np.power(10,self.variables['c32'][0]))
            self.distribute_channels("dend", "pbar_cav33", 1, 0,                        \
                                                              1,                        \
                                                              self.variables['c33'][1],      \
                                                              self.variables['c33'][2],      \
                                                              np.power(10,self.variables['c33'][0]))
        else:
            self.distribute_channels("dend", "gbar_naf", 1, 0.1, 0.9,   60.0,   10.0, float(par['gbar_naf_basal']['Value']))
            self.distribute_channels("dend", "gbar_kaf", 1,   1, 0.5,  120.0,  -30.0, float(par['gbar_kaf_basal']['Value']))
            #self.distribute_channels("dend", "gbar_kaf", 0, 1, 0, 0, 0, float(par['gbar_kaf_basal']['Value']))
            self.distribute_channels("dend", "gbar_kas", 2,   1, 9.0,  0.0, -5.0, float(par['gbar_kas_basal']['Value']))
            self.distribute_channels("dend", "gbar_kir", 0, 1, 0, 0, 0, float(par['gbar_kir_basal']['Value']))
            self.distribute_channels("soma", "gbar_kir", 0, 1, 0, 0, 0, float(par['gbar_kir_somatic']['Value']))
            self.distribute_channels("dend", "gbar_sk",  0, 1, 0, 0, 0, float(par['gbar_sk_basal']['Value']))
            self.distribute_channels("soma", "gbar_sk",  0, 1, 0, 0, 0, float(par['gbar_sk_basal']['Value']))
            self.distribute_channels("dend", "pbar_can", 0, 1, 0, 0, 0, 1e-7)
            self.distribute_channels("dend", "pbar_cav32", 1, 0, 1.0, 120.0, -30.0, 1e-7)
            self.distribute_channels("dend", "pbar_cav33", 1, 0, 1.0, 120.0, -30.0, 1e-8)
  
    def run_sim(cell_type,cell):
        try:
    
            # --- 1. Create a cell and other useful stuff
            dMSN_library = 'D1_71bestFit_updRheob.pkl'
            iMSN_library = 'D2_34bestFit_updRheob.pkl' 
    
            model_sets = ''
    
            if cell_type == 'D1':
                with open(dMSN_library, 'rb') as f:
                    model_sets  = pickle.load(f, encoding="latin1")
                    msn = d1msn
            elif cell_type == 'D2':
                with open(iMSN_library, 'rb') as f:
                    model_sets  = pickle.load(f, encoding="latin1")
                    msn = d2msn
    
            # cell_ID = 22
            # print (model_sets.keys())
            #cell_ID = 1
            if cell_ID not in model_sets:
                print ( " ---- cell_ID ---- " , cell_ID )
                return
                
            variables = model_sets[cell_ID]['variables'] 
            #cell = iMSN.iMSN(variables = variables) 
            cell = msn.MSN(variables = variables, morphology=morphology) 
    
            # for d in p.input_dends:
            #     cell.dendlist[d].nseg *=5
            # d_counter = 0
            # print ( " morphology is ", morphology)
            # print (cell.dendlist)
    
            for d in cell.dendlist:
    
                # print (" counter ", d)
                try:
                    cell.dendlist[d_counter].nseg *= 5
                except:
                    pass
                d_counter += 1
    
            # max_dist, sec_val = cell.max_dist()
            # sec_name = str(sec_val.name())
            # sec_num = int(sec_name[sec_name.find("[") + 1:sec_name.find("]")])
    
            # print("max -- " , max_dist, " - ", sec_num)
    
            # for sec in cell.dendlist:
            #     print(sec.name(), "%f, %f, %f, d = %.2f" % (h.distance(1, sec = sec), 
            #                                     h.distance(0, sec = sec), 
            #                                     h.distance(1, sec = sec) - h.distance(0, sec = sec),
            #                                     sec.diam))
    
            # for sec in cell.somalist:
            #     print(sec.name(), "%f, %f, %f, d = %.2f" % (h.distance(1, sec = sec), 
            #                                     h.distance(0, sec = sec), 
            #                                     h.distance(1, sec = sec) - h.distance(0, sec = sec),
            #                                     sec.diam))
    
    
            # # --- 2. Insert stimulation to cell
    
            # #independent_dends = [3, 5, 8, 12, 15, 22, 26, 35, 41, 47, 53, 57]
            # dend_record_list = [sec_num] #[3,4,9,10,21,22,24,26,35,36,51,52]
            # dend_stim_list = []#[3,4,9,10,35,36]                    
            # plateau_cluster_list = [sec_num]        
    
            # plateau_cluster_size = np.arange(1,5,1)
    
            # vs = []
            # vspine = []
            # vd = []
            # legend = []
    
            # max_vs = []
            # max_vspine = []
            # max_vd = []
            # g_nmda = []
            # i_nmda = []
    
            # sns.set(font_scale = 1.0)
            # sns.set_style('whitegrid')
            # fig_vs = plt.figure(); 
            # fig_vspine = plt.figure(); 
            # fig_vd = plt.figure();
            
            # ax_vs = fig_vs.add_subplot(111); ax_vs.set_ylabel('Vs (mV)'); ax_vs.set_xlabel('t (ms)')
            # ax_vspine = fig_vspine.add_subplot(111); ax_vspine.set_ylabel('Vspine (mV)'); ax_vspine.set_xlabel('t (ms)')
            # ax_vd = fig_vd.add_subplot(111); ax_vd.set_ylabel('Vd (mV)'); ax_vd.set_xlabel('t (ms)')
            # colors = sns.color_palette("coolwarm", plateau_cluster_size.max())
    
            # add_spine = 0
            # on_spine = 1
    
            # cell.insert_spines(plateau_cluster_list, p.cluster_start_pos, p.cluster_end_pos, num_spines = p.plateau_cluster_size_max)             
    
            # sns.set_style("ticks")
            # for num_syns in plateau_cluster_size:
    
            #     ex = pe.Spillover_Experiment('record_ca', cell)
            #     ex.insert_synapses('noise_SPN')
            #     ex.insert_synapses('my_spillover', plateau_cluster_list, deterministic = 0, 
            #                     num_syns = num_syns, add_spine = add_spine, on_spine = on_spine)
    
            #     ex.set_up_recording(dend_record_list)
            #     ex.simulate()
            #     tv = ex.tv.to_python()
            #     t = ex.t.to_python()
            #     legend.append("%d syns" % num_syns)
    
            #     if add_spine == 1 or on_spine == 1:
            #         vspine.append(ex.vspine[0].to_python())
            #         max_vspine.append(max(ex.vspine[0]))
            #         ax_vspine.plot(tv, ex.vspine[0].to_python(), color = colors[num_syns-1])
    
            #     vd.append(ex.vdlist[0].to_python())
            #     max_vd.append(max(ex.vdlist[0])) 
                
            #     max_vs.append(max(ex.vs))
            #     vs.append(ex.vs.to_python())
                    
            #     cell.esyn = []
            #     ex.estim = []
            #     ex.enc = []
            #     for s in cell.spines:
            #         s.syn_on = 0
            #     cell.isyn = []
            #     ex.istim = []
            #     ex.inc = []
            # #
            # vs_indices = []; vs_widths = []
            # vd_indices = []; vd_widths = []
            # vspine_indices = []; vspine_wid = []
            # for v in vs:
            #     vs_indices.append(ss.find_peaks(v))
            #     vs_widths.append(ss.peak_widths(v, (vs_indices[-1])[0], rel_height = 0.15))
    
            # for v in vd:
            #     vd_indices.append(ss.find_peaks(v))
            #     vd_widths.append(ss.peak_widths(v, (vd_indices[-1])[0] ,rel_height = 0.15))
                
            # for i in range(0, len(vs)):    
            #     if add_spine ==0 and on_spine ==0:    
            #         ax_vd.plot(tv, vd[i]);   
            #     ax_vs.plot(tv, vs[i], color = colors[i]); ax_vs.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
            #     ax_vd.plot(tv, vd[i], color = colors[i]); ax_vs.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
            # ax_vd.set_title("weight = %.2f, Cdur_factor = %d" % (p.weight, p.eCdur_factor))
                
            # sns.despine()
            # res_dict = {'t': t,
            #             'vs': vs,
            #             'vspine': vspine}
            # to_save = json.dumps(res_dict)
            # #filename = './results/data_spillover_steep.dat'
            # #with open(filename,'w', encoding = 'utf-8') as f:
            # #    json.dump(to_save, f)
            # plt.savefig("plots/" + cell_type + "_" + str(folder_number)+"_" + morphology.replace(".","_") + ".png")
        except:
            traceback.print_exc(file=sys.stdout)

    def create_sectionlists(self):
        self.all = []
        self.somalist = [] 
        self.nsomasec = 0                
        self.axonlist = [] 
        self.dendlist = [] 
        # print ( " ---- in create section list ------ ")
        for sec in h.allsec():
            # print ( " sec :::: ", sec.name() ) 
            self.all.append(sec) # needs to be a keyword argument when used with h.SectionList()
            if sec.name().find('soma') >= 0:
                self.neuron_id_list.append(self.neuron_id)
                print ( " ---- @@@@@@@ found soma !!!!!!!! ------ ")
                self.data_list.append([self.neuron_id,self.neuron_name,self.archive,self.note])
                self.somalist.append(sec)
                self.nsomasec += 1
            if sec.name().find('axon') >= 0:
                self.axonlist.append(sec)
            if sec.name().find('dend') >= 0:
                self.dendlist.append(sec)
        
    def distribute_channels(self, as1, as2, d3, a4, a5, a6, a7, g8):
        # print ("111 d3 ", d3, " a4 ", a4 , " a5 ", a5, " a6 ", a6, " a7 ", a7, " g8", g8 )
        print (" self.somalist[0] ", self.somalist[0])
        h.distance(sec=self.somalist[0][0])
        
        for sec in self.all:
            if sec.name().find(as1) >= 0:
                for seg in sec:
                    dist = h.distance(seg.x, sec=sec)
                    # print ("111 d3 ", d3, " a4 ", a4 , " a5 ", a5, " a6 ", a6, " a7 ", a7, " g8", g8 )
                    val = self.calculate_distribution(d3, dist, a4, a5, a6, a7, g8)
                    cmd = 'seg.%s = %g' % (as2, val)
                    exec(cmd)

    def distribute_channels_dend(self, as1, as2, d3, a4, a5, a6, a7, g8):
        h.distance(sec=self.somalist[0])
        for sec in as1:
            for seg in self.dendlist[sec]:
                dist = h.distance(seg.x, sec=self.dendlist[sec])
                print ("111 d3 ", d3, " a4 ", a4 , " a5 ", a5, " a6 ", a6, " a7 ", a7, " g8", g8 )
                val = self.calculate_distribution(d3, dist, a4, a5, a6, a7, g8)
                cmd = 'seg.%s = %g' % (as2, val)
                exec(cmd)

    def calculate_distribution(self, d3, dist, a4, a5, a6, a7, g8):
        # d3 is the distribution type:
        #     0 linear, 1 sigmoid, 2 exponential
        #     3 step for absolute distance (in microns)
        # dist is the somatic distance
        # a4-a7 are distribution parameters 
        # g8 is the maximal conductance
        if   d3 == 0: 
            value = a4 + a5*dist
        elif d3 == 1: 
            print (" dist ", dist)
            print (" a6 ", a6)

            print (" dist-a6 ", dist-a6)
            print (" a7 ", a7)
            print (" a5 ", a5)
            print (" a4 ", a4)
            value = 0
            try:
                value = a4 + a5/(1 + exp((dist-a6)/a7) )
            except:
                traceback.print_exc(file=sys.stdout)
                # pass
        elif d3 == 2: 
            value = a4 + a5*exp((dist-a6)/a7)
        elif d3 == 3:
            if (dist > a6) and (dist < a7):
                value = a4
            else:
                value = a5
                
        if value < 0:
            value = 0
            
        value = value*g8
        return value

    def get_dendrites(self, distance_to_soma = 80):
        distal_ind = []; proximal_ind = []; middle_ind = [];
        
        for ind, dend in enumerate(self.dendlist):
            if h.distance(0, sec = dend) >= distance_to_soma:
                distal_ind.append(ind)
            elif h.distance(1, sec = dend) <= distance_to_soma:
                proximal_ind.append(ind)
            else:
                middle_ind.append(ind)
                
        return distal_ind, proximal_ind, middle_ind

    
    def insert_spines(self, section_list, start_pos, end_pos, num_spines = 20):
        spine_step = 1.0/num_spines
        for sec in section_list:
            for i in range(0, num_spines):
                pos = end_pos - (end_pos - start_pos)*i*spine_step
                spine_name = 'spine_' + self.dendlist[sec].name() + '(' + str(pos) + ')'
                s = sp.Spine(self.dendlist[sec], spine_name)
                self.spines.append(s)            
                self.spines[-1].attach(self.dendlist[sec], pos, 0)

    def delete_spines(self):
        while self.spines != []:
            for s in self.spines:
                s.head = None                
                s.neck = None
                self.spines.remove(s)
        self.num_spines_on_dends = np.zeros(len(self.dendlist))
 
    def connect2target(self, target, thresh=10):
        """Make a new NetCon with this cell's membrane
        potential at the soma as the source (i.e. the spike detector)
        onto the target passed in (i.e. a synapse on a cell).
        Subclasses may override with other spike detectors."""
        nc = h.NetCon(self.soma(1)._ref_v, target, sec = self.soma)
        nc.threshold = thresh
        return nc
            
    def insert_synapse(self, syntype, sec, pos, add_spine = 0, on_spine = 0):
        if add_spine and on_spine:
            print("Arguments add_spine and on_spine can't simultaneously be 1")
            sys.exit(-1)

        if add_spine:
            s_ind = [int(si) for si in re.findall("\d+", sec.name())]
            s_ind = s_ind[0]
            self.num_spines_on_dends[s_ind] += 1

            spine_name = 'spine_' + sec.name() + '(' + str(pos) + ')'
            self.spines.append(sp.Spine(sec, spine_name))
            self.spines[-1].attach(sec, pos, 0)
            self.spines[-1].syn_on = 1
            sec = self.spines[-1].head
            s.spinepos = pos
            pos = 0.5
        
        if on_spine:
            empty_spines = [spine for spine in self.spines if (spine.parent == sec and spine.syn_on == 0)]
            
            if empty_spines == []:
                print("There are no empty spines on dendrite %s" % sec.name())
                # sys.exit(-1)
                return
            else:
                sec = empty_spines[0].head
                s.spinepos = pos
                pos = 0.5
                empty_spines[0].syn_on = 1
        
        syn = s.Synapse()
        syn.type = syntype
        syn.sec = sec
        syn.pos = pos        
        
        if syntype in ['expsyn', 'expsyn_plateau']:
            syn.obj = h.ExpSyn(sec(pos))            
            syn.obj.tau = p.esyn_tau
            syn.obj.e = p.e_esyn
            self.esyn.append(syn)
            return syn
        
        elif syntype == 'inhexpsyn' or syntype == 'inhexpsyn_plateau':
            syn.obj = h.InhExpSyn(sec(pos))
            if syntype == 'inhexpsyn':       
                syn.obj.tau = p.isyn_tau
            elif syntype == 'inhexpsyn_plateau':
                syn.obj.tau = p.isyn_plateau_tau
            syn.obj.e = p.e_gaba
            self.isyn.append(syn)
            return self.isyn[-1]
            
        elif syntype == 'exp2syn':
            syn.obj = h.Exp2Syn(sec(pos))
            syn.obj.e = p.e_esyn
            syn.obj.tau2 = p.tau2_exp2syn
            syn.obj.tau1 = p.tau1_exp2syn
            self.esyn.append(syn)
            return syn

        elif syntype == 'inhexp2syn':
            syn.obj = h.InhExp2Syn(sec(pos))
            syn.obj.e = p.e_gaba
            syn.obj.tau2 = p.tau2_inhexp2syn
            syn.obj.tau1 = p.tau1_inhexp2syn
            self.isyn.append(syn)
            return syn
            
        elif syntype == 'tmGlut':
            syn.obj = h.tmGlut(sec(pos))
            syn.obj.tau1_nmda = p.tau1_NMDA
            syn.obj.tau2_nmda = p.tau2_NMDA
            syn.obj.nmda_ratio = p.ratio_glutamate_syn
            syn.obj.U = 0.9
            syn.obj.tauF = 5.0
            self.esyn.append(syn)
            return syn

        elif syntype == 'glutamate' or syntype == 'glutamate_plateau':
            syn.obj = h.glutamate(sec(pos))
            syn.obj.mg = p.Mg
            syn.obj.eta = p.eta
            syn.obj.alpha = p.alpha
            syn.obj.tau1_nmda = p.tau1_NMDA
            syn.obj.tau2_nmda = p.tau2_NMDA
            syn.obj.ratio = p.ratio_glutamate_syn
            self.esyn.append(syn)
            return syn

        elif syntype in ['glutamate_ica_nmda', 'glutamate_xor_test'] :
            syn.obj = h.glutamate_ica_nmda(sec(pos))
            syn.obj.mg = p.Mg            
            syn.obj.eta = p.eta
            syn.obj.alpha = p.alpha
            syn.obj.tau1_nmda = p.tau1_NMDA
            syn.obj.tau2_nmda = p.tau2_NMDA
            
            syn.obj.w_ampa = p.gAMPAmax_plateau 
            syn.obj.w_nmda = p.gNMDAmax_plateau
            
            syn.obj.nmda_ca_fraction = p.nmda_ca_fraction
            self.esyn.append(syn)
            return syn

        elif syntype in ['AMPA' ,'AMPA_test', 'AMPA_pf', 'AMPA_stp']:
            if syntype in ['AMPA', 'AMPA_pf']:
                syn.obj = h.AMPA(sec(pos))
            elif syntype in ['AMPA_stp']:
                syn.obj = h.AMPA_stp(sec(pos))
                syn.obj.U = p.U
                syn.obj.u0 = p.u0
            elif syntype == 'AMPA_test':
                syn.obj = h.AMPA_test(sec(pos))
                syn.obj.weight = p.weight
            syn.obj.gmax = p.gmaxAMPA_spillover
            if syntype == 'AMPA_pf':
                syn.obj.gmax = p.gmaxAMPA_pf
            self.esyn.append(syn)
            return syn         
        
        elif syntype in [ 'NMDA', 'NMDA_test', 'NMDAe', 'NMDA_pf', 'NMDA_stp']:
            if syntype in ['NMDA', 'NMDA_pf']:
                syn.obj = h.NMDA(sec(pos))
            elif syntype in ['NMDA_stp']:
                syn.obj = h.NMDA_stp(sec(pos))
                syn.obj.U = p.U
                syn.obj.u0 = p.u0
            elif syntype == 'NMDA_test':
                syn.obj = h.NMDA_test(sec(pos))
            elif syntype ==  'NMDAe':
                syn.obj = h.NMDAe(sec(pos))
            syn.obj.mg = p.Mg
            syn.obj.eta = p.eta
            syn.obj.alpha = p.alpha
            syn.obj.Cdur = p.Cdur
            if syntype in ['NMDA', 'NMDA_test', 'NMDA_stp']:    
                syn.obj.gmax = p.gmaxNMDA_spillover
            elif syntype in ['NMDA_pf']:    
                syn.obj.gmax = p.gmaxNMDA_pf
            elif syntype in ['NMDAe']:
                syn.obj.gmax = p.gmaxNMDAe_spillover
                syn.obj.Cdur_init = p.eCdur_init
                syn.obj.Cdur_factor = p.eCdur_factor
                syn.obj.weight = p.exglu_weight
            syn.obj.Beta = p.Beta
            syn.obj.nmda_ca_fraction = p.nmda_ca_fraction
            if syntype == 'NMDA_test':
                syn.obj.weight = p.weight
            
            self.esyn.append(syn)
            return syn        

        elif syntype in ['adaptive_shom_AMPA', 'adaptive_shom_AMPA_stp']:
            if syntype in ['adaptive_shom_AMPA']:
                syn.obj = h.adaptive_shom_AMPA(sec(pos))
            else:
                syn.obj = h.adaptive_shom_AMPA_stp(sec(pos))
                syn.obj.U = p.U
                syn.obj.u0 = p.u0
                
            syn.obj.gmax = p.gmaxAMPA_spillover
            
            self.esyn.append(syn)
            return syn        

        elif syntype == 'adaptive_pf_AMPA':
            syn.obj = h.adaptive_pf_AMPA(sec(pos))
            syn.obj.gmax = p.gmaxAMPA_pf
            
            self.esyn.append(syn)
            return syn               
        
        elif syntype in ['adaptive_shom_NMDA','adaptive_shom_NMDA_stp','adaptive_my_shom_NMDA']:
            if syntype in ['adaptive_shom_NMDA']:
                syn.obj = h.adaptive_shom_NMDA(sec(pos))
            elif syntype in ['adaptive_shom_NMDA_stp']:
                syn.obj = h.adaptive_shom_NMDA_stp(sec(pos))
                syn.obj.U = p.U
                syn.obj.u0 = p.u0
            else:
                syn.obj = h.adaptive_my_shom_NMDA(sec(pos))
            
            syn.obj.mg = p.Mg
            syn.obj.eta = p.eta
            syn.obj.alpha = p.alpha
            syn.obj.gmax = p.gmaxNMDA_spillover
            syn.obj.Beta = p.Beta
            syn.obj.Cdur = p.Cdur
            syn.obj.nmda_ca_fraction = p.nmda_ca_fraction
            
            syn.obj.w0 = p.weight            
            syn.obj.learning_rate_w_LTP = p.learning_rate_w_LTP
            syn.obj.learning_rate_w_LTD = p.learning_rate_w_LTD
            syn.obj.learning_rate_thresh_LTP = p.learning_rate_thresh_LTP
            syn.obj.learning_rate_thresh_LTD = p.learning_rate_thresh_LTD
            syn.obj.learning_rate_thresh_KD_LTD = p.learning_rate_thresh_KD_LTD
            syn.obj.KD1 = p.KD1
            syn.obj.n1 = p.n1
            syn.obj.KD2 = p.KD2
            syn.obj.n2 = p.n2
            syn.obj.KD_LTD = p.KD_LTD
            syn.obj.n_LTD = p.n_LTD    

            self.esyn.append(syn)
            return syn  

        elif syntype == 'adaptive_pf_NMDA':
            syn.obj = h.adaptive_pf_NMDA(sec(pos))
            
            syn.obj.mg = p.Mg
            syn.obj.eta = p.eta
            syn.obj.alpha = p.alpha
            syn.obj.gmax = p.gmaxNMDA_pf
            syn.obj.Beta = p.Beta
            syn.obj.Cdur = p.Cdur_pf
            syn.obj.nmda_ca_fraction = p.nmda_ca_fraction
            
            syn.obj.w0 = p.weight            
            syn.obj.learning_rate_w_LTP = p.learning_rate_w_LTP
            syn.obj.learning_rate_w_LTD = p.learning_rate_w_LTD_pf
            syn.obj.learning_rate_thresh_LTP = p.learning_rate_thresh_LTP
            syn.obj.learning_rate_thresh_LTD = p.learning_rate_thresh_LTD
            syn.obj.KD_LTD = p.KD_LTD_pf
            syn.obj.n_LTD = p.n_LTD_pf
            
            self.esyn.append(syn)
            return syn  

        elif syntype == 'adaptive_NMDAe':
            syn.obj = h.adaptive_NMDAe(sec(pos))
            syn.obj.mg = p.Mg
            syn.obj.eta = p.eta
            syn.obj.alpha = p.alpha
            syn.obj.Erev = p.erev_NMDA
            syn.obj.gmax = p.gmaxNMDAe_spillover
            syn.obj.Beta = p.Beta
            syn.obj.Cdur = p.eCdur
            syn.obj.Cdur_init = p.eCdur_init
            syn.obj.Cdur_factor = p.eCdur_factor
            syn.obj.nmda_ca_fraction = p.nmda_ca_fraction
            
            self.esyn.append(syn)
            return syn           
        
        else: 
            print("From method cell.insert_synapse")
            print("Syntype '%s' not supported" % syntype)
            sys.exit(-1)

    def max_dist(self, axon_excluding=True):
        if not hasattr(self, 'somalist'):
            raise NotImplementedError("create_sectionlists() is not implemented or attribute somalist not defined")
        
        h.distance(sec=self.somalist[0])
        dmax = 0
        for sec in self.all:
            if axon_excluding and sec.name().find('axon') == 0: 
                continue
            dmax = max(dmax, h.distance(1, sec=sec))
        return dmax, sec
        
    def get_nsegs(self):
        """Returns the number of segments in the neuron model."""
        nsegs = 0
        for sec in self.all: 
            nsegs += sec.nseg
        return nsegs
        
    def set_nsegs(self):
        """Sets the number of segments in each section of the neuron model
        according to n = 2*int(L/40) + 1, where L is the length of the section."""
        for sec in self.all:
            sec.nseg = 2*int(sec.L/40.0)+1
        if hasattr(self, 'axonlist'):
            for sec in self.axonlist:
                sec.nseg = 2  # two segments in axon initial segment

    def total_dend_length(self):
        """Returns the total dendritic length."""
        total_length = 0             
        for dend in self.dendlist:
            total_length += dend.L
        return total_length
        
    def increase_dend_res(self, dend_list, mult):
        for d in dend_list:
            self.dendlist[d].nseg *= mult

sim = nmda_sim()
