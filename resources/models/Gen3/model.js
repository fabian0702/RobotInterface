/* Robot Web interface
* Copyright (C) BFH roboticsLab
* All rights reserved.
*/

import * as THREE from 'three';
import {T, updateT} from '../../../src/transform'
import {GLTFScaled} from '../../../src/GLTFScaled';
import { bfhMaterials } from '../../../src/materials';
import {RoundedBoxGeometry} from 'three/examples/jsm/geometries/RoundedBoxGeometry';
import {CoordinateSystem} from '../../../src/CoordinateSystem';


// Gripper length

// define coordinate systems
// For robot 3D model

const Difference = (arr1, arr2) => {
    const res = [];
    for(let i = 0; i < arr1.length; i++){
       const el = ((arr1[i] || 0) - (arr2[i] || 0));
       res[i] = el;
    };
    return res;
 };


export class Kinova{
    static base; 
    static link0;
    static link1;
    static link2;
    static link3;
    static link4;
    static link5;
    static link6;
    static link7;
    static cs;
    static cSystems;
    #GRIPPER_LENGTH = 0.10;

    

    // values from blender model
    #t01B = [0,0,0.15643];
    #t12B = [0,0.02473,0.2848];
    #t23B = [0,-0.01175,0.49517];
    #t34B = [0,-0.01813,0.70555];
    #t45B = [0,-0.024501,0.88197];
    #t56B = [0,-0.02467,1.0199];
    #t67B = [0,-0.02485,1.12582];

    

    #t01 = T(this.#t01B,[0,0,0]);
    #t12 = T(Difference(this.#t12B,this.#t01B),[0,0,0]);
    #t23 = T(Difference(this.#t23B,this.#t12B),[0,0,0]);
    #t34 = T(Difference(this.#t34B,this.#t23B),[0,0,0]);
    #t45 = T(Difference(this.#t45B,this.#t34B),[0,0,0]);
    #t56 = T(Difference(this.#t56B,this.#t45B),[0,0,0]);
    #t67 = T(Difference(this.#t67B,this.#t56B),[0,0,0]);

    forwardKin(q) {
        this.#t01 = updateT(this.#t01, this.#t01B,[0,0,q[0]]);
        this.#t12 = updateT(this.#t12, Difference(this.#t12B,this.#t01B),[0,q[1],0]);
        this.#t23 = updateT(this.#t23, Difference(this.#t23B,this.#t12B),[0,0,q[2]]);
        this.#t34 = updateT(this.#t34, Difference(this.#t34B,this.#t23B),[0,q[3],0]);
        this.#t45 = updateT(this.#t45, Difference(this.#t45B,this.#t34B),[0,0,q[4]]);
        this.#t56 = updateT(this.#t56, Difference(this.#t56B,this.#t45B),[0,q[5],0]);
        this.#t67 = updateT(this.#t67, Difference(this.#t67B,this.#t56B),[0,0,q[6]]);
    }

    updateVisibility(visible){
        var links = [this.link0, this.link1, this.link2, this.link3, this.link4, this.link5, this.link6,this.link7];
        links.forEach((value) => { value.visible = visible });
    }

    constructor(){
        this.base = new THREE.Group();
        // Offset base position to center it in canvas
        this.base.position.set(0.3, 0, 0);
        
        this.base.add(this.#t01);
        this.#t01.add(this.#t12);
        this.#t12.add(this.#t23);
        this.#t23.add(this.#t34);
        this.#t34.add(this.#t45);
        this.#t45.add(this.#t56);
        this.#t56.add(this.#t67);

        this.genLinks();
        var cords =  this.addCoordinateSystems();
        this.cs = cords.cs;
        this.cSystems = cords.cSystems;
    }
    
    
    

    genLinks(){
        // attach robot links
        this.link0 = new THREE.Group();
        this.link0.position.set(0.0, 0.0, 0.0);
        //this.link0.rotation.x = Math.PI;
        this.link0.add(new GLTFScaled("../files/Kinova/base.glb", 1));
        console.log('DONE')
        this.base.add(this.link0);
    
        this.link1 = new THREE.Group();
        this.link1.position.set(-this.#t01B[0], -this.#t01B[1], -this.#t01B[2]);
        //this.link1.rotation.x = Math.PI;
        this.link1.add(new GLTFScaled("../files/Kinova/joint1.glb", 1));
        this.#t01.add(this.link1);
    
        this.link2 = new THREE.Group();
        this.link2.position.set(-this.#t12B[0], -this.#t12B[1], -this.#t12B[2]);
        //this.link2.rotation.x = Math.PI;
        this.link2.add(new GLTFScaled("../files/Kinova/joint2.glb", 1));
        this.#t12.add(this.link2);
    
        this.link3 = new THREE.Group();
        this.link3.position.set(-this.#t23B[0], -this.#t23B[1], -this.#t23B[2]);
        //this.link3.rotation.x = Math.PI;
        this.link3.add(new GLTFScaled("../files/Kinova/joint3.glb", 1));
        this.#t23.add(this.link3);
    
        this.link4 = new THREE.Group();
        this.link4.position.set(-this.#t34B[0], -this.#t34B[1], -this.#t34B[2]);
        //this.link4.rotation.x = Math.PI;
        this.link4.add(new GLTFScaled("../files/Kinova/joint4.glb", 1));
        this.#t34.add(this.link4);

        this.link5 = new THREE.Group();
        this.link5.position.set(-this.#t45B[0], -this.#t45B[1], -this.#t45B[2]);
        //this.link5.rotation.x = Math.PI;
        this.link5.add(new GLTFScaled("../files/Kinova/joint5.glb", 1));
        this.#t45.add(this.link5);

        this.link6 = new THREE.Group();
        this.link6.position.set(-this.#t56B[0], -this.#t56B[1], -this.#t56B[2]);
        //this.link5.rotation.x = Math.PI;
        this.link6.add(new GLTFScaled("../files/Kinova/joint6.glb", 1));
        this.#t56.add(this.link6);

        this.link7 = new THREE.Group();
        this.link7.position.set(-this.#t67B[0], -this.#t67B[1], -this.#t67B[2]);
        //this.link6.rotation.x = Math.PI;
        this.link7.add(new GLTFScaled("../files/Kinova/joint7.glb", 1));
        this.#t67.add(this.link7);
    
        //return [link0,link2,link3,link4,link5];
    }
    
    createGripper(material){
    
        var gripper = new THREE.Group();
      
        var finger1 = new RoundedBoxGeometry(this.#GRIPPER_LENGTH, 0.005, 0.005, 2, 0.005);
        var mesh1 = new THREE.Mesh(finger1, material);
        mesh1.castShadow = true;
        mesh1.receiveShadow = true;
        mesh1.position.x = this.#GRIPPER_LENGTH/2.;
        mesh1.position.y = -0.0125;
        gripper.add(mesh1);
      
        var finger2 = new RoundedBoxGeometry(this.#GRIPPER_LENGTH, 0.005, 0.005, 2, 0.005);
        var mesh2 = new THREE.Mesh(finger1, material);
        mesh2.castShadow = true;
        mesh2.receiveShadow = true;
        mesh2.position.x = this.#GRIPPER_LENGTH/2.;
        mesh2.position.y = 0.0125;
        gripper.add(mesh2);
      
        var gripperBase = new THREE.CylinderGeometry(0.02, 0.02, 0.025, 32);
        var meshBase = new THREE.Mesh(gripperBase, material);
        meshBase.castShadow = true;
        meshBase.receiveShadow = true;
        meshBase.rotation.x = Math.PI / 2.0;
        meshBase.position.z = 0.01;
        gripper.add(meshBase);
      
        return gripper;
      }

      addCoordinateSystems(){
        // add coordinate systems
        var coordinateSystem0 = new CoordinateSystem(0.12, 0.004, "{B}", "X0", "Y0", " ");
        this.base.add(coordinateSystem0);

        var cs1 = new THREE.Group();
        cs1.matrixAutoUpdate = false;
        cs1.matrixWorldNeedsUpdate = true;
        var coordinateSystem1 = new CoordinateSystem(0.08, 0.004, "1", " ", " ", " ");
        cs1.add(coordinateSystem1);
        this.base.add(cs1);

        var cs2 = new THREE.Group();
        cs2.matrixAutoUpdate = false;
        cs2.matrixWorldNeedsUpdate = true;
        var coordinateSystem2 = new CoordinateSystem(0.08, 0.004, "2", " ", " ", " ");
        cs2.add(coordinateSystem2);
        this.base.add(cs2);

        var cs3 = new THREE.Group();
        cs3.matrixAutoUpdate = false;
        cs3.matrixWorldNeedsUpdate = true;
        var coordinateSystem3 = new CoordinateSystem(0.08, 0.004, "3", " ", " ", " ");
        cs3.add(coordinateSystem3);
        this.base.add(cs3);

        var cs4 = new THREE.Group();
        cs4.matrixAutoUpdate = false;
        cs4.matrixWorldNeedsUpdate = true;
        var coordinateSystem4 = new CoordinateSystem(0.08, 0.004, "4", " ", " ", " ");
        cs4.add(coordinateSystem4);
        this.base.add(cs4);

        var cs5 = new THREE.Group();
        cs5.matrixAutoUpdate = false;
        cs5.matrixWorldNeedsUpdate = true;
        var coordinateSystem5 = new CoordinateSystem(0.008, 0.004, "TCP");
        cs5.add(coordinateSystem5);
        this.base.add(cs5);

        var cSystems = [coordinateSystem1, coordinateSystem2, coordinateSystem3, coordinateSystem4, coordinateSystem5];
        var cs = [cs1, cs2, cs3, cs4, cs5];

        return {'cs': cs, 'cSystems': cSystems};
      }
}

