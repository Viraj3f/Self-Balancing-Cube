function output = pidnn_link(theta, ref)
    global pidnn
    output = pidnn.predict(theta/pi, ref);
end
