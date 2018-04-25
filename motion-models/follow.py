from models import Ellipse
import warnings
import itertools
import numpy as np
import statsmodels.api as sm
from models import Ellipse

warnings.filterwarnings("ignore")
def fit_modelsVARMAX(data,t_test=100,pds = None):
    if pds is None:
        pds = list(itertools.product(range(0, 5), range(0, 5)))
    best = []
    for param in pds:
        mse = np.infty
        try:
            model = sm.tsa.VARMAX(data[:-t_test],order =(param),
                                  enforce_stationarity=False,
                                 enforce_invertibility=False)
            results = model.fit(disp=False)
            mse = np.sum((results.forecast(t_test)-data[-t_test:])**2,1).mean()
            best.append((param,mse))
        except:
            continue
    best.sort(key=lambda x: x[1])
    result = None
    #fit final model
    for tup in best:
        param = tup[0]
        try:
            model = sm.tsa.VARMAX(data[:-t_test],order =(param),
                                  enforce_stationarity=False,
                                 enforce_invertibility=False)
            results = model.fit(disp=False)
            break
        except:
            #try next best
            continue
    return best




if __name__=='__main__':
    #generate data from desired model
    data = Ellipse(a=2,b=1).generate_data(N=400)
    #assume first i_seen datapoints have been observed
    i_seen = 300
    training = data[:i_seen]
    observations = data[i_seen:]

    #fit model
    model = fit_modelsVARMAX(training,t_test=int(i_seen/4))
    #forecast:
    preds = model.forecast(100)


