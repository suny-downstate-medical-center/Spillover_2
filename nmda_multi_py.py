from multiprocessing import Pool
import os
import time
import pandas as pd
import subprocess

def run_sim(neuron_id,neuron_name,archive,length,cell_type_val):
    # print(neuron_id,neuron_name,archive,length,cell_type_val)
    print ('python', 'nmda_plateaus_final.py',neuron_id, neuron_name, archive, length, cell_type_val)
    process = subprocess.Popen(['python', 'nmda_plateaus_final.py',neuron_id, neuron_name, archive, length, cell_type_val], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate() 
    print ( stdout, stderr)
    return "finished"

def process_cells():
    tic = time.time()
    pool = Pool(processes=os.cpu_count())
      
    df = pd.read_csv('soma.tsv',index_col=None, sep="\t")
    results = []
    for index,row in df.iterrows():

        neuron_id = row['neuron_id']
        neuron_name = row['neuron_name']
        print (neuron_name)
        archive = row['archive']
        length = row['length']
        cell_type_val = row['cell_type_val']
        
        if length < 4000: # discard those with length below a threshold
            continue
            
        # result = pool.apply_async(run_sim, args=(neuron_id,neuron_name,archive,length,cell_type_val))
        result = pool.apply(run_sim, args=(str(neuron_id),str(neuron_name),str(archive),str(length),str(cell_type_val)))

        results.append(result)

    pool.close()
    pool.join()

    # results = [r.get() for r in res]
    print(results)
    toc = time.time()
    print(f'Completed in {toc - tic} seconds')

if __name__ == '__main__':
        process_cells()
