#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }

        float tmpx = 0;
        if (num_nodes > 2) {
            tmpx = (end - start).norm();
            tmpx /= (num_nodes - 1);
        }
        Vector2D addv = (end - start).unit();
        for (int i=0; i<num_nodes; i++) {
            Vector2D subx = start + (i * tmpx) * addv;
            Mass *m = new Mass(subx,node_mass,false);
            masses.push_back(m);
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

        for (int i=1; i<num_nodes; i++) {
            Mass * a = masses[i-1], *b = masses[i];
            Spring *sp = new Spring(a,b,k);
            springs.push_back(sp);
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D tmpx = s->m2->position - s->m1->position;
            Vector2D f_huke_m2 = -1.0 * s->k * (tmpx.norm() - s->rest_length) * tmpx.unit();
            s->m2->forces += f_huke_m2;
            s->m1->forces += (-f_huke_m2);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position

                Vector2D temp_position = m->position;
                m->forces += gravity;

                Vector2D a = m->forces / m->mass;
                Vector2D bef_v = m->velocity;
                m->velocity += a * delta_t;

                // explicity method
                //m->position += (delta_t * bef_v);
                //
                // semi-implicity method
                m->position += (delta_t * m->velocity);
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D tmpx = s->m2->position - s->m1->position;
            Vector2D f_huke_m2 = -1.0 * s->k * (tmpx.norm() - s->rest_length) * tmpx.unit();
            s->m2->forces += f_huke_m2;
            s->m1->forces += (-f_huke_m2);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity;
                Vector2D a = m->forces / m->mass;

                //m->position += (temp_position - m->last_position + a * delta_t * delta_t);
                //m->last_position = m->position;
                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.5;
                m->position += ( (1-damping_factor)*(temp_position - m->last_position) + a*delta_t*delta_t );
                m->last_position = m->position;
            }
        }
    }
}
