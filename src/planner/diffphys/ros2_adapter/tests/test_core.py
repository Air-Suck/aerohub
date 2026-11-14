import ast
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np
import torch
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from uav_diffphys_planner.core import observation,decode

class OfficialDecoding(unittest.TestCase):
    def test_command_cone_preserves_vertical_force_and_rejects_inversion(self):
        action=np.array([7.,0,2.,0,0.,0])
        q,force=decode(action,np.eye(3),np.eye(3),np.array([10.,0,0]),0.,.5,max_command_tilt_rad=.5)
        up=Rotation.from_quat(q).as_matrix()[:,2]
        self.assertAlmostEqual(np.arccos(up[2]),.5)
        self.assertAlmostEqual(force*up[2],9.80665/9.8*.5*15)
        safe=np.array([1.,0,0.,0,0.,0])
        raw=decode(safe,np.eye(3),np.eye(3),np.array([10.,0,0]),0.,.5)
        limited=decode(safe,np.eye(3),np.eye(3),np.array([10.,0,0]),0.,.5,max_command_tilt_rad=.5)
        np.testing.assert_array_equal(raw[0],limited[0]);self.assertEqual(raw[1],limited[1])
        with self.assertRaises(ValueError):
            decode([0,0,0,0,-20,0],np.eye(3),np.eye(3),np.array([10.,0,0]),0.,.5,max_command_tilt_rad=.5)
        # Validate the final command after the existing cone limiter. A large
        # horizontal request can become safe without changing vertical force.
        q,force=decode([15,0,0,0,2,0],np.eye(3),np.eye(3),np.array([10.,0,0]),0.,.5,max_command_tilt_rad=.5)
        self.assertLess(force/15,.85)
        self.assertAlmostEqual(force*Rotation.from_quat(q).as_matrix()[2,2],(9.80665+2)/9.8*.5*15)
        with self.assertRaises(ValueError):
            decode([0,0,0,0,10,0],np.eye(3),np.eye(3),np.array([10.,0,0]),0.,.5,max_command_tilt_rad=.5)
    def test_height_lookahead_preserves_speed_heading_and_near_goal(self):
        rotation=Rotation.from_euler('xyz',[.1,-.2,1.1]).as_matrix()
        position=np.array([2.,3.,2.]);goal=np.array([-37.,232.,6.])
        legacy,level=observation(position,rotation,np.zeros(3),goal,speed=7.)
        guided,_=observation(position,rotation,np.zeros(3),goal,speed=7.,goal_lookahead_m=20.)
        a,b=level@legacy[3:6],level@guided[3:6]
        self.assertGreater(b[2],a[2]*5)
        self.assertAlmostEqual(np.linalg.norm(b),7.)
        np.testing.assert_allclose(a[:2]/np.linalg.norm(a[:2]),b[:2]/np.linalg.norm(b[:2]))
        for speed in [8.,10.,13.]:
            for lookahead in [5.,10.,20.]:
                state,_=observation(position,rotation,np.zeros(3),goal,speed=speed,
                    goal_lookahead_m=lookahead)
                self.assertAlmostEqual(np.linalg.norm(state[3:6]),speed)
        for target in [position,position+[2,3,1],position+[0,0,4]]:
            direct,_=observation(position,rotation,np.zeros(3),target,speed=7.)
            near,_=observation(position,rotation,np.zeros(3),target,speed=7.,goal_lookahead_m=20.)
            np.testing.assert_array_equal(direct,near)
        for bad in [-1.,float('inf'),float('nan')]:
            with self.assertRaises(ValueError):
                observation(position,rotation,np.zeros(3),goal,goal_lookahead_m=bad)

    def test_nonidentity_against_official_ast(self):
        root=Path(__file__).resolve().parents[4]
        tree=ast.parse((Path(os.environ.get('DIFFPHYS_MODEL_DIR',str(root/'inference/models/diffphys')))/'artifacts/swarm/eval.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        loop=next(n for n in main.body if isinstance(n,ast.While))
        selected=[];active=False
        for n in loop.body:
            if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Tuple) and any(isinstance(t,ast.Name) and t.id=='v_setpoint' for t in n.targets[0].elts):active=True
            if active:
                if isinstance(n,ast.For):break
                selected.append(n)
        self.assertTrue(selected)
        code=compile(ast.Module(body=selected,type_ignores=[]),'official-control-decode','exec')
        random=np.random.default_rng(7)
        for index in range(30):
            matrix=Rotation.from_euler('xyz',[.1,-.15,index*.1]).as_matrix()
            position=np.array([2.,-3.,1.]);velocity=np.array([.3,-.2,.1]);goal=np.array([10.,12.,2.])
            state,level=observation(position,matrix,velocity,goal)
            action=random.normal(0,.15,6)
            ctx=dict(torch=torch,R=torch.tensor(level)[None],env_R=torch.tensor(matrix)[None],
                     action=torch.tensor(action)[None],B=1,ctl_error=torch.zeros(1,3),
                     local_v=torch.tensor(velocity@level)[None],p_target=torch.tensor(goal)[None],last_p=torch.tensor(position)[None])
            exec(code,ctx)
            q,force=decode(action,level,matrix,goal-position,float(state[2]),.5)
            official=np.column_stack([ctx[k][0].numpy() for k in ['forward_vec','left_vec','up_vec']])
            np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),official,atol=1e-8)
            self.assertAlmostEqual(force,ctx['throttle'].item()/9.8*.5*15,places=8)
            np.testing.assert_array_equal(state[6:9],[0.,0.,1.])

    def test_high_speed_against_official_ast(self):
        root=Path(__file__).resolve().parents[4]
        tree=ast.parse((Path(os.environ.get('DIFFPHYS_MODEL_DIR',str(root/'inference/models/diffphys')))/'artifacts/high_speed/PlannerBase.py').read_text())
        callback=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='callback_depth')
        selected=[];active=False
        for n in callback.body:
            if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Tuple) and any(
                    isinstance(t,ast.Name) and t.id=='act' for t in n.targets[0].elts):active=True
            if active:
                if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute) and n.value.func.attr=='publish':
                    break
                selected.append(n)
        self.assertTrue(selected)
        code=compile(ast.Module(body=selected,type_ignores=[]),'official-high-speed-control-decode','exec')
        random=np.random.default_rng(11)
        fake_self=SimpleNamespace(forward=None,h=None)
        previous_forward=None
        for index in range(30):
            matrix=Rotation.from_euler('xyz',[.08,-.12,index*.07]).as_matrix()
            position=np.array([2.,-3.,1.]);velocity=np.array([.3,-.2,.1])
            goal=position+np.array([100.,0.,0.])
            state,level=observation(position,matrix,velocity,goal,speed=8.,margin=.2,full_attitude=True)
            self.assertEqual(state[9],.2)
            np.testing.assert_allclose(state[6:9],matrix[2],atol=1e-12)
            target_velocity=level@state[3:6]
            action=random.normal(0,.18,6)
            if fake_self.forward is None:
                fake_self.forward=torch.tensor(level[:,0])
                previous_forward=level[:,0]
            fake_self.model=lambda depth,current_state,hidden: (
                torch.tensor(action)[None], torch.zeros(1,192))
            ctx=dict(torch=torch,R=torch.tensor(level),env_R=torch.tensor(matrix),
                     self=fake_self,depth=torch.zeros(1,1,12,16),
                     state=torch.tensor(state),
                     local_v=torch.tensor(velocity@level),p_target=torch.tensor(goal),
                     last_p=torch.tensor(position),target_v=torch.tensor(target_velocity),
                     matrix_to_quaternion=lambda value: SimpleNamespace(tolist=lambda: [1.,0.,0.,0.]))
            exec(code,ctx)
            q,force=decode(action,level,matrix,goal-position,float(state[2]),.5,
                           gravity=9.81,heading_vector=target_velocity,vertical_correction=0.,
                           previous_forward=previous_forward)
            previous_forward=Rotation.from_quat(q).as_matrix()[:,0]
            official=np.column_stack([ctx['self'].forward.numpy(),ctx['left_vec'].numpy(),ctx['up_vec'].numpy()])
            np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),official,atol=1e-8)
            self.assertAlmostEqual(force,ctx['thrust'].item()/9.8*.5*15,places=8)

    def test_invalid_control_rejected(self):
        with self.assertRaises(ValueError):decode([float('nan')]*6,np.eye(3),np.eye(3),np.ones(3),0,.5)
        with self.assertRaises(ValueError):decode([0,0,0,0,-30,0],np.eye(3),np.eye(3),np.ones(3),0,.5)

if __name__=='__main__':unittest.main()
