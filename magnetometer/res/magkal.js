const DEG = Math.PI/180;

function calcHdgDip(m1, m2, m3) {
    let hdg = Math.atan2(-m2, m1) / DEG;
    if (hdg < 0) { hdg += 360}
    return {hdg: hdg,
        dip: Math.atan2(m3, Math.sqrt(m1*m1 + m2*m2)) / DEG
    }
}

let width = 400, height=400,
    margin = {top: 20, right: 2, bottom: 20, left: 40};

let connectedIndicator = function(el) {
    return function(connected) {
        if (connected) {
            el.className = "connected";
            el.innerText = "connected";
        } else {
            el.className = "disconnected";
            el.innerText = "disconnected";
        }
    }
};

let updateTable = (function() {
    function formatVar(k, v) {
        let fmt;
        switch (k) {
            case "HDG":
            case "HDGRaw":
            case "DIP":
            case "DIPRaw":
                fmt = ".1f";
                break;
            case "S1":
            case "S2":
            case "S3":
                fmt = ".2f";
                break;
            default:
                fmt = "+.0f";
        }
        return d3.format(fmt)(v);
    }

    return function(data) {
        for (let f in data) {
            d3.select("#"+f).text(formatVar(f, data[f]));
        }
    }
})();

// Draw Magnetometer cross-sections
function updateMagXS(ax, ay) {
    let col, mx, my, ox, oy, sx, sy, lLim=-1, rLim=1, tLim=1, bLim=-1;

    switch (ax+ay) {
        case 3:
            col = "Blue";
            break;
        case 4:
            col = "Green";
            break;
        case 5:
            col = "Red";
    }

    let x = d3.scale.linear()
        .domain([lLim, rLim])
        .range([0, width-margin.left-margin.right]);

    let y = d3.scale.linear()
        .domain([bLim, tLim])
        .range([height-margin.top-margin.bottom, 0]);

    let xAxis = d3.svg.axis()
        .scale(x)
        .orient("bottom");

    let yAxis = d3.svg.axis()
        .scale(y)
        .orient("left");

    let svg = d3.select("#m"+ax+"m"+ay).append("svg")
        .attr("width", width)
        .attr("height", height)
        .append("g");

    let xAxisLine = svg.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0," + y(0) + ")")
        .call(xAxis);

    xAxisLine.append("text")
        .attr("x", 6)
        .attr("dx", ".71em")
        .style("text-anchor", "end")
        .text("m"+ax);

    let yAxisLine = svg.append("g")
        .attr("class", "y axis")
        .attr("transform", "translate(" + x(0) + ",0)")
        .call(yAxis);

    yAxisLine.append("text")
        .attr("y", 6)
        .attr("dy", ".71em")
        .style("text-anchor", "end")
        .text("m"+ay);

    let dots = svg.append("g");

    let ctr = svg.append("circle")
        .attr("class", "center")
        .attr("r", 2)
        .attr("cx", x(0))
        .attr("cy", y(0));

    let crc = svg.append("ellipse")
        .attr("class", "ellipse")
        .attr("cx", x(0))
        .attr("cy", y(0))
        .attr("rx", 0)
        .attr("ry", 0);

    let vec = svg.append("line")
        .attr("class", "pointer")
        .attr("x1", x(0))
        .attr("y1", y(0))
        .attr("x2", x(0))
        .attr("y2", y(0));

    return function(data) {
        mx = data['M'+ax];
        my = data['M'+ay];
        ox = data["O"+ax];
        oy = data["O"+ay];
        sx = data["S"+ax];
        sy = data["S"+ay];

        if (mx < lLim) {
            lLim = mx;
            x.domain([lLim, rLim]);
            xAxis.scale(x)
        }
        if (mx > rLim) {
            rLim = mx;
            x.domain([lLim, rLim]);
            xAxis.scale(x)
        }
        if (my < bLim) {
            bLim = my;
            y.domain([bLim, tLim]);
            yAxis.scale(y)
        }
        if (my > tLim) {
            tLim = my;
            y.domain([bLim, tLim]);
            yAxis.scale(y)
        }

        dots.append("circle")
            .attr("class", "dot")
            .attr("r", 2)
            .attr("cx", x(mx))
            .attr("cy", y(my))
            .style("fill", col);

        ctr
            .attr("cx", x(ox))
            .attr("cy", y(oy));

        crc
            .attr("cx", x(ox))
            .attr("cy", y(oy))
            .attr("rx", (x(ox+sx) - x(ox-sx))/2)
            .attr("ry", (y(oy-sy) - y(oy+sy))/2);

        vec
            .attr("x1", x(ox))
            .attr("y1", y(oy))
            .attr("x2", x(mx))
            .attr("y2", y(my))
    }

}

function makeRollingPlot(el, v) {
    const TMAX = 10;

    let D0 = [], bLim = -1, tLim = 1;

    let x = d3.scale.linear()
        .domain([0, TMAX])
        .range([0, width-margin.left-margin.right]);

    let y = d3.scale.linear()
        .domain([bLim, tLim])
        .range([height-margin.top-margin.bottom, 0]);

    let xAxis = d3.svg.axis()
        .scale(x)
        .orient("bottom");

    let yAxis = d3.svg.axis()
        .scale(y)
        .orient("left");

    let getLine = function(dim) {
        return d3.svg.line()
            .x(function (d, i) {
                return x(i / 10);
            })
            .y(function (d, i) {
                return y(d[v + dim]);
            });
    };

    let svg = d3.select("#"+el)
        .append("svg")
        .attr("width", width)
        .attr("height", height)
        .append("g")
        .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    svg.append("defs").append("clipPath")
        .attr("id", el+"Clip")
        .append("rect")
        .attr("width", width-margin.left-margin.right)
        .attr("height", height-margin.top-margin.bottom);

    let xAxisLine = svg.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0," + y(0) + ")")
        .call(xAxis);

    let yAxisLine = svg.append("g")
        .attr("class", "y axis")
        .call(yAxis);

    yAxisLine.append("text")
        .attr("transform", "rotate(-90)")
        .attr("y", 6)
        .attr("dy", ".71em")
        .style("text-anchor", "end")
        .text(el);

    let xpath = svg.append("g")
        .attr("clip-path", "url(#"+el+"Clip)")
        .append("path")
        .attr("class", "line x")
        .datum(D0)
        .attr("d", getLine("1"));

    let ypath = svg.append("g")
        .attr("clip-path", "url(#"+el+"Clip)")
        .append("path")
        .attr("class", "line y")
        .datum(D0)
        .attr("d", getLine("2"));

    let zpath = svg.append("g")
        .attr("clip-path", "url(#"+el+"Clip)")
        .append("path")
        .attr("class", "line z")
        .datum(D0)
        .attr("d", getLine("3"));

    return function(data) {
        if (data[v+"1"] < bLim) {
            bLim = data[v+"1"];
        }
        if (data[v+"2"] < bLim) {
            bLim = data[v+"2"];
        }
        if (data[v+"3"] < bLim) {
            bLim = data[v+"3"];
        }
        if (data[v+"1"] > tLim) {
            tLim = data[v+"1"];
        }
        if (data[v+"2"] > tLim) {
            tLim = data[v+"2"];
        }
        if (data[v+"3"] > tLim) {
            tLim = data[v+"3"];
        }
        y.domain([bLim, tLim]);
        xAxisLine.attr("transform", "translate(0," + y(0) + ")").call(xAxis);
        yAxis.scale(y);
        yAxisLine.call(yAxis);

        D0.push(data);
        xpath.attr("d", getLine("1"))
            .attr("transform", null)
            .transition()
            .duration(100)
            .ease("linear")
            .attr("transform", "translate(" + x(TMAX-D0.length/10-0.1) + ",0)");
        ypath.attr("d", getLine("2"))
            .attr("transform", null)
            .transition()
            .duration(100)
            .ease("linear")
            .attr("transform", "translate(" + x(TMAX-D0.length/10-0.1) + ",0)");
        zpath.attr("d", getLine("3"))
            .attr("transform", null)
            .transition()
            .duration(100)
            .ease("linear")
            .attr("transform", "translate(" + x(TMAX-D0.length/10-0.1) + ",0)");
        if (D0.length>10*TMAX) {
            D0.shift();
        }
    }

}

