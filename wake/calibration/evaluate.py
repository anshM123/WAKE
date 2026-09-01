import numpy as np
def regression_metrics(truth:np.ndarray,predicted:np.ndarray)->dict[str,float]:
    error=np.abs(np.asarray(truth)-np.asarray(predicted));return {"mae":float(error.mean()),"p90_error":float(np.percentile(error,90))}
def detection_metrics(truth:np.ndarray,probability:np.ndarray,threshold:float=.5)->dict[str,float]:
    y=np.asarray(truth,bool);p=np.asarray(probability)>=threshold;tp=np.sum(y&p);fp=np.sum(~y&p);fn=np.sum(y&~p);tn=np.sum(~y&~p)
    return {"precision":float(tp/max(1,tp+fp)),"recall":float(tp/max(1,tp+fn)),"false_positive_rate":float(fp/max(1,fp+tn))}
