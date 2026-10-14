from pathlib import Path
import sys,random,math,json,copy,time,subprocess,os
ROOT=Path(__file__).resolve().parents[3]
if len(sys.argv)>1 and sys.argv[1]=='worker':
 BASE=Path(__file__).resolve().parent
else:
 if len(sys.argv)!=2:
  raise SystemExit('Usage: python '+Path(__file__).name+' NEW_OUTPUT_DIRECTORY')
 BASE=Path(sys.argv[1]).resolve()
 BASE.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(ROOT/'src/m4_global_planner'))
from m4_global_planner.graph import Graph,Node,Edge,MotionMode
from m4_global_planner.global_planner import GlobalPlanner
from m4_global_planner.astar import astar,dijkstra
from m4_global_planner.benchmark_algorithms import LPAStar,DStarLite,bidirectional_dijkstra,bidirectional_astar,weighted_astar,safe_heuristic_scale

def fixture(seed,zero=True):
 rng=random.Random(seed);g=Graph()
 for i in range(7):g.add_node(Node(i,rng.randrange(20),rng.randrange(20),rng.randrange(5),MotionMode.WALK if i%2 else MotionMode.FLY))
 for a in g.nodes:
  for b in g.nodes:
   if a!=b and rng.random()<.3:g.add_edge(Edge(a,b,rng.choice([0,0.25,1,3,9] if zero else [.25,1,3,9])))
 return g

def oracle(g):
 n=len(g.nodes);d=[[math.inf]*n for _ in range(n)]
 for i in range(n):d[i][i]=0
 for a,out in g.edges.items():
  for b,e in out.items():d[a][b]=min(d[a][b],e.cost)
 for k in range(n):
  for a in range(n):
   for b in range(n):d[a][b]=min(d[a][b],d[a][k]+d[k][b])
 return d

def audit(g,a,b,r,expected):
 assert r['success']==math.isfinite(expected),(r,expected)
 assert math.isclose(r['total_cost'],expected,rel_tol=1e-10,abs_tol=1e-10),(r,expected)
 if r['success']:
  assert r['path'][0]==a and r['path'][-1]==b
  total=math.fsum(g.edges[x][y].cost for x,y in zip(r['path'],r['path'][1:]))
  assert math.isclose(total,expected,rel_tol=1e-10,abs_tol=1e-10),(total,expected,r)
 else:assert not r['path']

def dynamic(seed,zero):
 results=[]
 for typ in (LPAStar,DStarLite):
  g=fixture(seed,zero);p=typ(g,0,6);a=0
  for step in range(13):
   if step:
    rng=random.Random(seed*100+step);u,v=rng.sample(range(7),2);c=rng.choice([None,0,.25,1,4] if zero else [None,.25,1,4])
    p.update_edge(u,v,c)
    if typ is DStarLite and step%3==0:a=(a+1)%7;p.move_start(a)
   r=p.plan();audit(g,a,6,r,oracle(g)[a][6]);results.append({'algorithm':typ.__name__,'step':step,'cost':r['total_cost']})
 return results

if len(sys.argv)>1 and sys.argv[1]=='worker':
 try:print(json.dumps({'status':'pass','rows':dynamic(int(sys.argv[2]),sys.argv[3]=='zero')}))
 except Exception as e:print(json.dumps({'status':'fail','error':repr(e)}))
 sys.exit()
counts={};fail=[];began=time.perf_counter()
for seed in range(20):
 g=fixture(seed);ref=oracle(g);scale=safe_heuristic_scale(g)
 methods={'dijkstra':lambda a,b:dijkstra(g,a,b),'astar':lambda a,b:astar(g,a,b,scale),'bidirectional_dijkstra':lambda a,b:bidirectional_dijkstra(g,a,b),'bidirectional_astar':lambda a,b:bidirectional_astar(g,a,b),'weighted_1':lambda a,b:weighted_astar(g,a,b,1,scale),'LPAStar':lambda a,b:LPAStar(g,a,b).plan(),'DStarLite':lambda a,b:DStarLite(g,a,b).plan()}
 for name,fn in methods.items():
  for a in g.nodes:
   for b in g.nodes:
    try:audit(g,a,b,fn(a,b),ref[a][b]);counts[name]=counts.get(name,0)+1
    except Exception as e:fail.append({'seed':seed,'algorithm':name,'start':a,'goal':b,'error':repr(e)})
print('Static checks',counts,'failures',len(fail),flush=True)
workers=[]
for mode in ('positive','zero'):
 for seed in range(12):
  try:
   r=subprocess.run([sys.executable,__file__,'worker',str(seed),mode],text=True,capture_output=True,timeout=4,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
   data=json.loads(r.stdout);data.update(seed=seed,mode=mode);workers.append(data)
  except subprocess.TimeoutExpired:workers.append({'status':'timeout','seed':seed,'mode':mode})
  except Exception as e:workers.append({'status':'error','seed':seed,'mode':mode,'error':repr(e)})
print('Dynamic workers:',[(r['mode'],r['seed'],r['status']) for r in workers],flush=True)
out={'static_pass_counts':counts,'static_failures':fail,'dynamic':workers,'elapsed_s':time.perf_counter()-began,'oracle':'Floyd-Warshall independent of production search','purpose':'Correctness checks, not speed comparison','seeds':list(range(20))}
(BASE/'independent_results_fixed.json').write_text(json.dumps(out,indent=2))

