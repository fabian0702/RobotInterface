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
export class RS20_Model{
    static base; 
    static link0;
    static link1;
    static link2;
    static link3;
    static link4;
    static link5;
    static cs;
    static cSystems;
    #GRIPPER_LENGTH = 0.10;
    #L0Z = 0.262;
    #L1Z = 0.118;
    #L2X = 0.110;
    #L3X = 0.110;
    #L3Z = -this.#L1Z;
    #L4X = 0.085;
    #L4Z = 0.000;
    #L5Z = -0.0225;

    #t01 = T([0,0,this.#L0Z],[0,0,0]);
    #t12 = T([0,0,this.#L1Z], [0,0,0]);
    #t23 = T([this.#L2X,0,0], [0,0,0]);
    #t34 = T([this.#L3X, 0, this.#L3Z],[0,0,0]);
    #t45 = T([this.#L4X,0,this.#L4Z],[0,0,0]);

    forwardKin(q) {
        this.#t01 = updateT(this.#t01, [0,0,this.#L0Z+q[0]],[0,0,0]);
        this.#t12 = updateT(this.#t12, [0,0,this.#L1Z], [0,0,q[1]]);
        this.#t23 = updateT(this.#t23, [this.#L2X,0,0], [0,0,q[2]]);
        this.#t34 = updateT(this.#t34, [this.#L3X, 0, this.#L3Z],[0,0,q[3]]);
    }

    updateVisibility(visible){
        var links = [this.link0, this.link2, this.link3, this.link4, this.link5];
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

        this.genLinks();
        var cords =  this.addCoordinateSystems();
        this.cs = cords.cs;
        this.cSystems = cords.cSystems;
    }
    
    
    

    genLinks(){
        // attach robot links
        this.link0 = new THREE.Group();
        this.link0.position.set(0.0, 0.0, 0.0);
        this.link0.rotation.x = Math.PI / 2.0;
        this.link0.add(new GLTFScaled("../files/rs20/rs20-0.glb", 0.001, bfhMaterials.gray));
        console.log('DONE')
        this.base.add(this.link0);
    
        this.link2 = new THREE.Group();
        this.link2.position.set(0.0, 0, -this.#L0Z - this.#L1Z);
        this.link2.rotation.x = Math.PI / 2.0;
        this.link2.add(new GLTFScaled("../files/rs20/rs20-2.glb", 0.001, bfhMaterials.plastic_orange));
    
        var cylinder = new THREE.CylinderGeometry(0.035, 0.035, 0.2, 32);
        var cylinderMesh = new THREE.Mesh(cylinder, bfhMaterials.plastic_black);
        cylinderMesh.castShadow = true;
        cylinderMesh.receiveShadow = true;
        cylinderMesh.position.set(0,this.#L0Z + this.#L1Z-0.1,0)
        this.link2.add(cylinderMesh);
    
        this.#t12.add(this.link2);
    
        this.link3 = new THREE.Group();
        this.link3.position.set(-this.#L2X, 0, -this.#L0Z - this.#L1Z);
        this.link3.rotation.x = Math.PI / 2.0;
        this.link3.add(new GLTFScaled("../files/rs20/rs20-3.glb", 0.001,bfhMaterials.orange));
        this.#t23.add(this.link3);
    
        this.link4 = new THREE.Group();
        this.link4.position.set(-this.#L2X-this.#L3X, 0, -this.#L0Z);
        this.link4.rotation.x = Math.PI / 2.0;
        this.link4.add(new GLTFScaled("../files/rs20/rs20-4.glb", 0.001,bfhMaterials.silver));
        this.#t34.add(this.link4);
    
        this.link5 = new THREE.Group();
        this.link5.position.set(0, 0, this.#L5Z);
        this.link5.add(this.createGripper(bfhMaterials.silver));
        this.#t34.add(this.link5);
    
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
        var coordinateSystem5 = new CoordinateSystem(this.#GRIPPER_LENGTH-0.03, 0.004, "TCP");
        cs5.add(coordinateSystem5);
        this.base.add(cs5);

        var cSystems = [coordinateSystem1, coordinateSystem2, coordinateSystem3, coordinateSystem4, coordinateSystem5];
        var cs = [cs1, cs2, cs3, cs4, cs5];

        return {'cs': cs, 'cSystems': cSystems};
      }
}

