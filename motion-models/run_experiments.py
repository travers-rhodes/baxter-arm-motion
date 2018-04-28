


import warnings
import itertools
import pandas as pd
import numpy as np
import statsmodels.api as sm
import matplotlib.pyplot as plt
import cPickle as pkl
from multiprocessing import Pool
warnings.filterwarnings("ignore")
def run_experiments(idx):
        train = int(np.ceil(idx/3.0))*2
        val = idx-train
        [modelresult,param,mse] = fit_modelsVARMAX(data,t_train=train,horizon=val)
        preds = modelresult.forecast(horizon)
        eucl_dist = np.sqrt(np.sum((preds-data[idx:idx+horizon])**2,1))#2-norm
        return (eucl_dist,param,mse)


def fit_modelsVARMAX(data,t_train=300,horizon=100,pds = [(1,0),(2,0),(3,0),(3,1),(3,2)]):
    best = [None,None,np.infty]
    for param in pds:
        mse = np.infty
        try:
            model = sm.tsa.VARMAX(data[:t_train],order =(param),
                                  error_cov_type = 'diagonal',
                                measurement_error=True,
                                  enforce_stationarity=False,
                                 enforce_invertibility=False)
            results = model.fit(disp=False)
            #np.isnan(results.aic)
            #print('VARMAX{} - AIC:{}'.format(param, results.aic))
            mse = np.sum((results.forecast(horizon)-data[t_train:t_train+horizon])**2,1).mean()
            print 'VARMAX{} mse: {}'.format(param,mse)
            if mse<best[2]:
                model = sm.tsa.VARMAX(data[:t_train+horizon],order =(param),
                                  enforce_stationarity=False,
                                 enforce_invertibility=False)
                results = model.fit(disp=False)
                best = [results,param,mse]
        except Exception as e: 
            print 'error {}'.format(param)
            print(e)
            continue
    return best


for _d in xrange(2,7):
    fname = 'data%d-30sps.npy'%_d
    print fname
    data = np.load(fname)
    tstart = 30
    tend = 300
    indexes = range(tstart,tend)
    horizon = 60
    param = (3,0)
    results = np.empty([len(indexes), horizon])
    results[:] = np.nan
    prevparams = None
    count = 0    
    pool = Pool(processes=40)
    fullresults = pool.map(run_experiments, indexes) 
    #fullresults = [run_experiments(x) for x in indexes]
    pool.close()
    pkl.dump(fullresults,open('results%d.pkl'%_d,'wb'))
