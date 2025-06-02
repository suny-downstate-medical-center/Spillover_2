from neuron import h
from math import exp
import numpy as np
import json
import neuron_cls as n
import spine as sp
import parameters as p

mod        = "./mod/"
params  = "./params_iMSN.json"
morphology = "./morphology/WT-iMSN_P270-09_1.01_SGA2-m1.swc"

try:
    h.load_file('stdlib.hoc')
except:
    pass
try:
    h.load_file('import3d.hoc')
except:
    pass
try:
    h.nrn_load_dll(mod + 'x86_64/.libs/libnrnmech.so')       
except:
    pass
# ======================= the MSN class ==================================================

def create_morphology(morphology):    
    Import = h.Import3d_SWC_read()
    Import.input(morphology)
    imprt = h.Import3d_GUI(Import, 0)
    imprt.instantiate(None)
    h.define_shape()            
    h.celsius = 35
    v_init = -80

morphology = './morphology/msn_morphologies/morphology_4/D2/iSPN-Sham-A1S2N2.CNG.swc'

create_morphology(morphology)