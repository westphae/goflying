var magCalSimple = function() {
    var mrange = [[Infinity, -Infinity], [Infinity, -Infinity], [Infinity, -Infinity]];

    return function(data) {
        for (i = 0; i < 3; i++) {
            mrange[i] = [Math.min(mrange[i][0], data['M' + (i+1)]), Math.max(mrange[i][1], data['M' + (i+1)])];
            data['O' + (i+1)] = (mrange[i][1] + mrange[i][0])/2;
            data['S' + (i+1)] = (mrange[i][1] - mrange[i][0])/2;
            if (data['S' + (i+1)] == 0) {
                data['S' + (i+1)] = Infinity;
            }
        }
    }
};

function calcHdgDip(m1, m2, m3) {
    var hdg = Math.atan2(-m2, m1)*180/Math.PI;
    if (hdg < 0) { hdg += 360}
    return {hdg: hdg,
            dip: Math.atan2(m3, Math.sqrt(m1*m1 + m2*m2))*180/Math.PI
    }
}

function smoother(k) {
    var data = {};

    return function(newdata) {
        for (var d in newdata) {
            if (isFinite(newdata[d])) {
                if (!(d in data)) {
                    data[d] = newdata[d];
                } else {
                    data[d] = k * data[d] + (1 - k) * newdata[d];
                }
            }
        }
        return data;
    }
}
