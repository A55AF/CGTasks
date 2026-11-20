using CGUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class DivideAndConquer : Algorithm
    {
        public Tuple<int, int> GetUpperTangent(List<Point> L, List<Point> R, int mostL, int mostR)
        {
            int upperL = mostL, upperR = mostR;
            int szL = L.Count(), szR = R.Count();
            bool done = false;
            while (!done)
            {
                done = true;

                while (szL > 1)
                {
                    int next = (upperL + 1) % szL;
                    var turn = HelperMethods.CheckTurn(new Line(R[upperR], L[upperL]), L[next]);
                    if (turn == Enums.TurnType.Right
                        || (turn == Enums.TurnType.Colinear && HelperMethods.PointOnSegment(L[upperL], R[upperR], L[next])))
                    {
                        upperL = next;
                        done = false;
                    }
                    else
                    {
                        break;
                    }
                }

                while (szR > 1)
                {
                    int prev = (upperR - 1 + szR) % szR;
                    var turn = HelperMethods.CheckTurn(new Line(L[upperL], R[upperR]), R[prev]);
                    if (turn == Enums.TurnType.Left
                        || (turn == Enums.TurnType.Colinear && HelperMethods.PointOnSegment(R[upperR], L[upperL], R[prev])))
                    {
                        upperR = prev;
                        done = false;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            return new Tuple<int, int>(upperL, upperR);
        }

        public Tuple<int, int> GetLowerTangent(List<Point> L, List<Point> R, int mostL, int mostR)
        {
            int lowerL = mostL, lowerR = mostR;
            int szL = L.Count(), szR = R.Count();
            bool done = false;
            while (!done)
            {
                done = true;

                while (szL > 1)
                {
                    int prev = (lowerL - 1 + szL) % szL;
                    var turn = HelperMethods.CheckTurn(new Line(R[lowerR], L[lowerL]), L[prev]);
                    if (turn == Enums.TurnType.Left
                        || (turn == Enums.TurnType.Colinear && HelperMethods.PointOnSegment(L[lowerL], R[lowerR], L[prev])))
                    {
                        lowerL = prev;
                        done = false;
                    }
                    else
                    {
                        break;
                    }
                }

                while (szR > 1)
                {
                    int next = (lowerR + 1) % szR;
                    var turn = HelperMethods.CheckTurn(new Line(L[lowerL], R[lowerR]), R[next]);
                    if (turn == Enums.TurnType.Right
                        || (turn == Enums.TurnType.Colinear && HelperMethods.PointOnSegment(R[lowerR], L[lowerL], R[next])))
                    {
                        lowerR = next;
                        done = false;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            return new Tuple<int, int>(lowerL, lowerR);
        }

        public List<Point> Merge(List<Point> L, List<Point> R)
        {
            int szL = L.Count;
            int szR = R.Count;

            int mostL = 0;
            for (int i = 1; i < szL; i++)
            {
                if (L[i].X > L[mostL].X || (L[i].X == L[mostL].X && L[i].Y > L[mostL].Y)) mostL = i;
            }

            int mostR = 0;
            for (int i = 1; i < szR; i++)
            {
                if (R[i].X < R[mostR].X || (R[i].X == R[mostR].X && R[i].Y < R[mostR].Y)) mostR = i;
            }

            var upperTangent = GetUpperTangent(L, R, mostL, mostR);
            var lowerTangent = GetLowerTangent(L, R, mostL, mostR);

            int upperL = upperTangent.Item1, upperR = upperTangent.Item2;
            int lowerL = lowerTangent.Item1, lowerR = lowerTangent.Item2;

            var merged = new List<Point>();
            int cur = upperL;
            merged.Add(L[cur]);
            while (cur != lowerL)
            {
                cur = (cur + 1) % szL;
                merged.Add(L[cur]);
            }

            cur = lowerR;
            merged.Add(R[cur]);
            while (cur != upperR)
            {
                cur = (cur + 1) % szR;
                merged.Add(R[cur]);
            }

            return merged;
        }

        public List<Point> Hull(List<Point> points)
        {
            if (points.Count <= 3)
            {
                if (points.Count == 3)
                {
                    var turn = HelperMethods.CheckTurn(new Line(points[0], points[1]), points[2]);
                    if (turn == Enums.TurnType.Colinear)
                        return new List<Point> { points[0], points[2] };
                    if (turn == Enums.TurnType.Right)
                        return new List<Point> { points[0], points[2], points[1] };
                }
                return points;
            }

            int splitSizeL, splitSizeR;
            if (points.Count() % 2 == 0)
            {
                splitSizeL = splitSizeR = points.Count() / 2;
            }
            else
            {
                splitSizeL = (points.Count() - 1) / 2;
                splitSizeR = splitSizeL + 1;
            }
            var L = points.GetRange(0, splitSizeL);
            var R = points.GetRange(splitSizeL, splitSizeR);

            var HL = Hull(L);
            var HR = Hull(R);

            return Merge(HL, HR);
        }
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            var uniquePoints = new List<Point>();
            points.Sort((a, b) =>
            {
                int cmpX = a.X.CompareTo(b.X);
                return cmpX != 0 ? cmpX : a.Y.CompareTo(b.Y);
            });
            for (int i = 0; i < points.Count(); i++)
            {
                if (i > 0 && points[i].Equals(points[i - 1])) continue;
                uniquePoints.Add(points[i]);
            }

            outPoints = Hull(uniquePoints);
        }

        public override string ToString()
        {
            return "Convex Hull - Divide & Conquer";
        }

    }
}
