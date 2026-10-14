from pathlib import Path
import sys,os,random,math,time,json,csv,statistics
ROOT=Path(__file__).resolve().parents[3]
if len(sys.argv)>1 and sys.argv[1]=='worker':
 BASE=Path(__file__).resolve().parent
else:
 if len(sys.argv)!=2:
  raise SystemExit('Usage: python '+Path(__file__).name+' NEW_OUTPUT_DIRECTORY')
 BASE=Path(sys.argv[1]).resolve()
 BASE.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(ROOT/'src/m4_global_planner'))
sys.path.insert(0,str(ROOT/'benchmarks'))
from fixtures import _grid,_multimodal_demo
from m4_global_planner.graph import Graph,Node,Edge,MotionMode
from m4_global_planner.global_planner import GlobalPlanner

def ref(g,a,b):
 d={n:math.inf for n in g.nodes};d[a]=0
 for _ in range(len(g.nodes)-1):
  changed=False
  for u,out in g.edges.items():
   for v,e in out.items():
    val=d[u]+e.cost
    if val<d[v]:d[v]=val;changed=True
  if not changed:break
 return d[b]

cases=[]
g=_grid(40);cases.append(('open_grid',g,0,1599))
g=_grid(40)
# Three alternating barriers, each with one opening. These are supplied graph cuts,
# not collision or clearance claims about a physical robot.
for column,gap in [(9,35),(19,4),(29,35)]:
 for u in g.nodes:
  x,y=u%40,u//40
  for v in list(g.edges[u]):
   nx,ny=v%40,v//40
   if min(x,nx)==column and max(x,nx)==column+1 and not(y==ny==gap):del g.edges[u][v]
cases.append(('alternating_barriers',g,0,1599))
g=_grid(40)
for n in list(g.nodes):g.nodes[n]=Node(n,mode=MotionMode.WALK)
cases.append(('coordinates_omitted',g,0,1599))
g=_grid(40);g.edges[0][1]=Edge(0,1,0);cases.append(('zero_cost_guidance',g,0,1599))
g=Graph();rng=random.Random(71)
for i in range(300):g.add_node(Node(i,rng.randrange(100),rng.randrange(100),rng.randrange(20),MotionMode.WALK if i%3 else MotionMode.FLY))
for i in range(300):
 targets={min(i+1,299)}|set(rng.sample(range(300),3))
 for j in sorted(targets-{i}):g.add_edge(Edge(i,j,rng.choice([.25,1,4,9])))
cases.append(('directed_mixed_modes',g,0,299))
rows=[];summaries=[]
for name,g,a,b in cases:
 expected=ref(g,a,b)
 for rep in range(7):
  for method in (('astar','dijkstra') if rep%2 else ('dijkstra','astar')):
   p=GlobalPlanner(search_method=method)
   start=time.perf_counter_ns();p.set_graph(g,rep);accepted=time.perf_counter_ns();r=p.plan(a,b);end=time.perf_counter_ns()
   assert r['success'] and math.isclose(r['total_cost'],expected,rel_tol=1e-10)
   assert math.isclose(math.fsum(e.cost for e in r['edges']),expected,rel_tol=1e-10)
   plan_ms=(end-accepted)/1e6
   rows.append(dict(case=name,method=method,repetition=rep,nodes=len(g.nodes),edges=sum(map(len,g.edges.values())),cost=expected,accept_ms=(accepted-start)/1e6,search_ms=r['search_ms'],audit_and_result_ms=plan_ms-r['search_ms'],total_ms=(end-start)/1e6,expanded=r['nodes_expanded']))
 for method in ['dijkstra','astar']:
  group=[x for x in rows if x['case']==name and x['method']==method]
  summary={'case':name,'method':method,'runs':7,'nodes':len(g.nodes),'edges':sum(map(len,g.edges.values())),'cost':expected,'expanded':group[0]['expanded']}
  for field in ('accept_ms','search_ms','audit_and_result_ms','total_ms'):
   vals=[x[field] for x in group];summary[field+'_median']=statistics.median(vals)
   if field=='total_ms':summary['total_mean_ms']=statistics.mean(vals);summary['total_sd_ms']=statistics.stdev(vals);summary['total_min_ms']=min(vals);summary['total_max_ms']=max(vals)
  summaries.append(summary)
for name,data in [('paired_raw.csv',rows),('paired_summary.csv',summaries)]:
 with (BASE/name).open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
for row in summaries:print(row['case'],row['method'],'cost',round(row['cost'],4),'search',round(row['search_ms_median'],3),'total',round(row['total_ms_median'],3),'expanded',row['expanded'])
