Stage(function(stage) {

    var Math = Stage.Math;

    var spaceWidth = 36;
    var spaceHeight = 20;
    var THRESHOLD = 9;
    var boidNum = 30;


    var world;
    var boidBodies = [];

    function init() {
        world = new p2.World({
            gravity : [ 0, 0]
        });

        addBoids();
    }
    function start() {
        uiStatus();
        play();
        uiStart();
    }

    function play() {

    }

    function tick() {

        for (var i = 0; i < boidBodies.length; i++) {
            var boidBody = boidBodies[i];
            var ang = Math.atan2(boidBody.velocity[1], boidBody.velocity[0]);
            ang = normalizeAngle(ang);
            boidBody.angle = ang - Math.PI/2;
            boidBody.tempV = [0,0];
            boidBody.separate();
            boidBody.align1();
            boidBody.cohere();
            boidBody.center1();
            boidBody.velocity[0] += boidBody.tempV[0];
            boidBody.velocity[1] += boidBody.tempV[1];
            wrap(boidBody);
        }

    }

    function addBoids() {
        while (boidBodies.length) {
            var boidBodyOld = boidBodies.shift();
            world.removeBody(boidBodyOld);
    }

        for (var i = 0; i < boidNum; i++) {

            var a = Math.random(0, 2 * Math.PI) + Math.PI/2;
            var r = Math.random(0, 9);
            var magnitude = Math.random(0,2) ;

            var boidBody = makeBoidBody(
                {
                    x : r * Math.cos(a),
                    y : r * Math.sin(a)
                },
                [
                    magnitude * Math.cos(a),
                    magnitude * Math.sin(a)
                ]
                );
            world.addBody(boidBody);
            boidBodies.push(boidBody);
        }
    }

    function normalizeAngle(angle){
        angle = angle % (2*Math.PI);
        if(angle < 0){
            angle += (2*Math.PI);
        }
        return angle;
    }

    function wrap(body) {
        var p = body.position;
        p[0] = Math.rotate(p[0], -spaceWidth / 2 , spaceWidth / 2);
        p[1] = Math.rotate(p[1], -spaceHeight / 2, spaceHeight / 2);

         if (body.velocity[0] > 2) {
            body.velocity[0] = 2;
        }
        if (body.velocity[1] > 2) {
            body.velocity[1] = 2;
        }
    }

    function makeBoidBody(from, v) {

        var path = [ [ -0.15, -0.15 ], [ 0, -0.1 ], [ 0.15, -0.15 ], [ 0, 0.2 ] ];

        var boidBody;
        boidBody = new p2.Body({
            mass : 1,
            position : [from.x, from.y],
            velocity : [v[0],v[1]],
            angularVelocity: 0
        }).noDamping();

        boidBody.fromPolygon(path);

        return boidBody;
    }

    p2.Body.prototype.tempV = [0,0];

    p2.Body.prototype.noDamping = function() {
        this.damping = this.angularDamping = 0;
        return this;
    };

    p2.Body.prototype.within_fov = function(body) {
        var v2 = [0,0];
        var posA = this.position;
        var posB = body.position;
        v2 = p2.vec2.sub(v2, posA,posB);
        var ml;
        ml = p2.vec2.dot(this.velocity, v2);
        var lenA = p2.vec2.length(this.velocity);
        var lenB = p2.vec2.length(v2);
        var cos_angle =  ml / (lenA * lenB);
        if (Math.acos(cos_angle) < Math.PI * 0.75) {
            return true;
        } else {
            return false;
        }
    };

    p2.Body.prototype.near_by = function(body, threshold) {
        if (this.id === body.id) {
            return false;
        }
        var  result = (p2.vec2.dist(this.position,body.position) < threshold);
        return (result && this.within_fov(body));
    };

    p2.Body.prototype.separate = function() {
        var body,v = 0;
        for (var i = 0; i < boidBodies.length; i++) {
            body = boidBodies[i];
//            console.log("this.nearby = "+this.near_by(body,2));
             if (this.near_by(body, 2)) {
                 v += p2.vec2.dist(this.position,body.position);
             }
        }
//                 console.log("v = "+v+"\nbody.id = "+body.id);
        this.tempV[0] += v/500;
        this.tempV[1] += v/500;
    };

    p2.Body.prototype.align1 = function() {
        var average_velocity = [0,0],nearby = 0,body;

        for(var i = 0; i < boidBodies.length; i++) {
            body = boidBodies[i];
            if (this.near_by(body, THRESHOLD)) {
                average_velocity[0] += body.velocity[0];
                average_velocity[1] += body.velocity[1];
                nearby += 1;
            }
        }
        if (nearby == 0) {
            nearby = 1;
        }
        average_velocity[0] /= nearby;
        average_velocity[1] /= nearby;

        this.tempV[0] += (average_velocity[0] - this.velocity[0])/500;
        this.tempV[1] += (average_velocity[1] - this.velocity[1])/500;
//        console.log("this.id = "+this.id+"\naver = "+average_velocity+"\nnear = "+nearby+"\nthis.v = "+this.velocity);
    };

    p2.Body.prototype.cohere = function() {
        var average_position = [0,0],nearby = 0;
        for (var  i = 0; i < boidBodies.length; i++) {
            var  body = boidBodies[i];
            if (this.near_by(body,THRESHOLD)) {
                average_position[0] += body.position[0];
                average_position[1] += body.position[1];
                nearby += 1;
            }
            if (nearby == 0) {
                nearby = 1;
            }
        }
            average_position[0] /= nearby;
            average_position[1] /= nearby;
//console.log("aver_pos = "+average_position+"\nnearby = "+nearby);
        this.tempV[0] += (average_position[0] - this.position[0])/500;
        this.tempV[1] += (average_position[1] - this.position[1])/500;


    };

    p2.Body.prototype.center1 = function() {

        var r = 400;
        var posi = [0,0];
        posi[0] = this.position[0];
        posi[1] = this.position[1];

        this.tempV[0] -= (posi[0] + 0)/r;
        this.tempV[1] -= (posi[1] + 0)/r;
//        console.log("this.vel = "+this.velocity+"\nthis.pos = "+this.position);
//        console.log("posi = "+posi[0]);
    };

    var ui = {};

    stage.on('viewport', function(size) {
        ui.meta.pin({
            scaleMode : 'in-pad',
            scaleWidth : size.width,
            scaleHeight : size.height
        });
        ui.p2.pin({
            scaleMode : 'in-pad',
            scaleWidth : size.width,
            scaleHeight : size.height
        });
    });
    init();

    ui.p2 = new Stage.P2(world, {
        lineColor : '#fff',
        fillColor : ''
    }).pin({
            handle : -0.5,
            width : spaceWidth,
            height : spaceHeight
        }).appendTo(stage);

    stage.tick(tick);

    ui.meta = Stage.create().pin({
        width : 1000,
        height : 1000
    }).appendTo(stage);

    ui.status = Stage.string('text').pin({
        align : 0,
        offset : 20
    }).appendTo(ui.meta);

    ui.gameover = Stage.string('text').value('Game Over').pin({
        align : 0.5,
        scale : 1.6
    }).appendTo(ui.meta);

    function uiStart() {
        ui.gameover.hide();
    }

    function uiEnd() {
        ui.gameover.show();
    }

    function uiStatus() {
        ui.status.value('Biot');
    }

    start();
});

Stage({
    textures : {
        text:function(d) {
            d += '';
            return Stage.canvas(function(ctx) {
                var  ratio = 2;
                this.size(16,24,ratio);
                ctx.scale(ratio, ratio);
                ctx.font = 'bold 24px monospace';
                ctx.fillStyle = '#ddd';
                ctx.textBaseline = 'top';
                ctx.fillText(d,0,1);
            });
        }
    }
});
