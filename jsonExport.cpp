#include "JsonExport.h"
#include <fstream>
#include <sstream>

// Small helper: write a list of Vec2 as a JSON array of [x, y] pairs.
static void WritePointArray(std::ostream& out, const std::vector<Vec2>& pts) {
    out << "[";
    for (size_t i = 0; i < pts.size(); ++i) {
        out << "[" << pts[i].x << "," << pts[i].y << "]";
        if (i + 1 < pts.size()) out << ",";
    }
    out << "]";
}

void ExportToJSON(const std::string& filepath, const std::vector<CorridorResult>& corridors) {
    std::ofstream out(filepath);
    out << "[\n";

    for (size_t c = 0; c < corridors.size(); ++c) {
        const CorridorResult& cr = corridors[c];
        out << "  {\n";
        out << "    \"name\": \"" << cr.name << "\",\n";

        out << "    \"start\": [" << cr.start.x << "," << cr.start.y << "],\n";
        out << "    \"goal\": [" << cr.goal.x << "," << cr.goal.y << "],\n";

        out << "    \"portals\": [";
        for (size_t i = 0; i < cr.portals.size(); ++i) {
            out << "[[" << cr.portals[i].left.x << "," << cr.portals[i].left.y << "],["
                << cr.portals[i].right.x << "," << cr.portals[i].right.y << "]]";
            if (i + 1 < cr.portals.size()) out << ",";
        }
        out << "],\n";

        out << "    \"funnelPath\": ";
        WritePointArray(out, cr.funnelPath);
        out << ",\n";

        out << "    \"rubberPath\": ";
        WritePointArray(out, cr.rubberPath);
        out << ",\n";

        out << "    \"splinePath\": ";
        WritePointArray(out, cr.splinePath);
        out << "\n";

        out << "  }";
        if (c + 1 < corridors.size()) out << ",";
        out << "\n";
    }

    out << "]\n";
    out.close();
}