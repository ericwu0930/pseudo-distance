function failure_rate = calFailureRates(node,n_free,n_bor,n_failure)
if node(end)==0
    failure_rate = n_failure*n_free/(n_bor+n_free);
else
    failure_rate = n_failure*n_bor/(n_bor+n_free);
end
end