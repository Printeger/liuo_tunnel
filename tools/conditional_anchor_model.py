"""Independent D1 range/observed-plane model. Contains no truth Anchor loader."""
import numpy as np


class AnchorProblem:
    def __init__(self, ids, ta, aa, patches=()):
        self.ids = list(ids)
        index = {key: i for i, key in enumerate(ids)}
        self.ta_index = np.array([index[row['anchor']] for row in ta], dtype=int)
        self.tags = np.asarray([row['tag'] for row in ta], dtype=float).reshape(-1, 3)
        self.ta_range = np.array([row['range'] for row in ta])
        self.ta_sigma = np.sqrt([row['variance'] for row in ta])
        self.aa_first = np.array([index[row['first']] for row in aa], dtype=int)
        self.aa_second = np.array([index[row['second']] for row in aa], dtype=int)
        self.aa_range = np.array([row['range'] for row in aa])
        self.aa_sigma = np.sqrt([row['variance'] for row in aa])
        self.patches = [(index[row['anchor']], row) for row in patches]

    def residual_jacobian(self, flat):
        anchors = np.asarray(flat).reshape(-1, 3)
        n = len(self.ta_index)
        m = len(self.aa_first)
        residual = np.empty(n+m+len(self.patches))
        jacobian = np.zeros((len(residual), anchors.size))
        delta = anchors[self.ta_index]-self.tags
        distance = np.linalg.norm(delta, axis=1)
        residual[:n] = (distance-self.ta_range)/self.ta_sigma
        derivative = delta/np.maximum(distance[:, None], 1e-15)/self.ta_sigma[:, None]
        for j in range(3):
            jacobian[np.arange(n), 3*self.ta_index+j] = derivative[:, j]
        delta = anchors[self.aa_first]-anchors[self.aa_second]
        distance = np.linalg.norm(delta, axis=1)
        residual[n:n+m] = (distance-self.aa_range)/self.aa_sigma
        derivative = delta/np.maximum(distance[:, None], 1e-15)/self.aa_sigma[:, None]
        for j in range(3):
            jacobian[n+np.arange(m), 3*self.aa_first+j] = derivative[:, j]
            jacobian[n+np.arange(m), 3*self.aa_second+j] = -derivative[:, j]
        for k, (index, patch) in enumerate(self.patches):
            local = np.asarray(patch['R']).T@(anchors[index]-patch['p'])
            geometry = patch['geometry']
            normal = np.asarray(geometry['normal'])
            residual[n+m+k] = (normal@local-geometry['center_offset']-patch['offset'])/geometry['sigma']
            jacobian[n+m+k, 3*index:3*index+3] = normal@np.asarray(patch['R']).T/geometry['sigma']
        return residual, jacobian

    def objective(self, flat):
        residual, jacobian = self.residual_jacobian(flat)
        return .5*residual@residual, jacobian.T@residual

    def feasible_probe(self, flat):
        """Finite independent directional falsification, never a uniqueness proof."""
        point = np.asarray(flat, dtype=float).reshape(-1)
        base, gradient = self.objective(point)
        directions = list(np.eye(len(point)))+list(-np.eye(len(point)))
        if np.linalg.norm(gradient) > 0:
            directions.append(-gradient/np.linalg.norm(gradient))
        records = []
        for step in (1e-3, 1e-4):
            changes = []
            for direction in directions:
                trial = point+step*direction
                margins = self.domain(trial)
                if len(margins) and min(margins) < -1e-7:
                    continue
                residual, _ = self.residual_jacobian(trial)
                before, _ = self.residual_jacobian(point)
                # Difference of squares avoids subtracting large total costs.
                change = np.sum((residual-before).astype(np.longdouble)*(residual+before).astype(np.longdouble))/2
                changes.append(float(change))
            records.append(dict(step_m=step, feasible_probes=len(changes), minimum_cost_change=min([0.]+changes)))
        return dict(cost=float(base), gradient_norm=float(np.linalg.norm(gradient)), probes=records,
                    limitation='No finite direction finding descent does not prove stationarity, especially at active boundaries.')

    def domain(self, flat):
        """Production finite patch + final floor/height association, margins >=0.

        These are observed geometric support guards, not extra residuals or
        surveyed installation heights. They cannot lower the reported cost.
        """
        anchors = np.asarray(flat).reshape(-1, 3)
        margins = []
        for index, patch in self.patches:
            point = np.asarray(patch['R']).T@(anchors[index]-patch['p'])
            g = patch['geometry']
            along = np.dot(g['tangent'], point)
            side = int(patch['offset'] >= 0)
            floor = g['floors'][side]
            floor_along = np.dot(floor['longitudinal_axis'], point)
            delta = point-floor['center']
            floor_margin = 3*np.sqrt(floor['variance']*(1+np.dot(floor['along'], delta)**2/floor['along_spread']
                                                    +np.dot(floor['across'], delta)**2/floor['across_spread']))
            margins.extend([along-g['minimum_longitudinal'], g['maximum_longitudinal']-along,
                            floor_along-floor['minimum_longitudinal'], floor['maximum_longitudinal']-floor_along,
                            g['maximum_height']+.25-np.dot(g['up'], point),
                            np.dot(floor['normal'], delta)+floor_margin])
        return np.asarray(margins)

    def range_seeds(self, anchor):
        """Eight deterministic near/mirror starts from noisy TA only."""
        selected = self.ta_index == self.ids.index(anchor)
        points, ranges = self.tags[selected], self.ta_range[selected]
        if not len(points):
            return None
        center = points.mean(axis=0)
        centered = points-center
        # This rank-aware linear estimate is only a seed, never a measurement.
        matrix = 2*(centered-centered[0])
        rhs = np.sum(centered**2, axis=1)-np.sum(centered[0]**2)-ranges**2+ranges[0]**2
        seed = np.linalg.lstsq(matrix, rhs, rcond=1e-8)[0]
        _, _, basis = np.linalg.svd(centered, full_matrices=False)
        if basis.shape != (3, 3):
            basis = np.eye(3)
        radial2 = np.median(ranges**2-np.sum((seed-centered)**2, axis=1))
        radius = max(.5, np.sqrt(max(0., radial2)))
        return np.array([center+seed+radius*(np.cos(a)*basis[-1]+np.sin(a)*basis[-2])
                         for a in np.arange(8)*np.pi/4])
