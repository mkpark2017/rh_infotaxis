function resp = sensorMeasureCallback(~,req,resp)
    if(req.force_pull)
        resp.success = true;
        resp.param_received = 0;
    end
end