using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Schema;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class ExtremeSegments : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            var uniquePointsTmp = new HashSet<Tuple<double, double>>();
            var pts = new List<Point>();

            foreach (Point p in points)
            {
                var cur = new Tuple<double, double>(p.X, p.Y);
                if (uniquePointsTmp.Contains(cur)) continue;
                uniquePointsTmp.Add(cur);
                pts.Add(p);
            }

            if (pts.Count <= 1)
            {
                outPoints = pts;
                return;
            }

            for (int i = 0; i < pts.Count; i++)
            {
                Point s = pts[i];
                for (int j = i + 1; j < pts.Count; j++)
                {
                    Point e = pts[j];
                    bool ok = true;
                    Line cur = new Line(s, e);
                    int cntr = 0, cntl = 0;
                    for (int k = 0; k < pts.Count; k++)
                    {
                        if (k == i || k == j) continue;
                        Point a = pts[k];
                        var dir = HelperMethods.CheckTurn(cur, a);
                        if (dir == Enums.TurnType.Left) cntl++;
                        else if (dir == Enums.TurnType.Right) cntr++;
                        else if (!HelperMethods.PointOnSegment(a, s, e))
                        {
                            ok = false;
                            break;
                        }
                        if (cntl > 0 && cntr > 0)
                        {
                            ok = false;
                            break;
                        }
                    }

                    if (ok)
                    {
                        outLines.Add(cur);
                        outPoints.Add(s);
                        outPoints.Add(e);
                    }
                }
            }

            uniquePointsTmp = new HashSet<Tuple<double, double>>();
            pts = new List<Point>();

            foreach (Point p in outPoints)
            {
                var cur = new Tuple<double, double>(p.X, p.Y);
                if (uniquePointsTmp.Contains(cur)) continue;
                uniquePointsTmp.Add(cur);
                pts.Add(p);
            }
            outPoints = pts;
        }

        public override string ToString()
        {
            return "Convex Hull - Extreme Segments";
        }
    }
}
