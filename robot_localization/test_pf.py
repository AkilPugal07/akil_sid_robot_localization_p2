import math
import pytest
from pf import ParticleFilter, Particle


class FakeTF:
    """Stands in for TFHelper. Poses are passed around as (x, y, theta) tuples."""

    def convert_pose_to_xy_and_theta(self, pose):
        """Convert a pose to (x, y, theta) tuple."""
        return pose

    def angle_normalize(self, a):
        """Normalize an angle to the range [-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))

    def angle_diff(self, a, b):
        """Compute the difference between two angles, normalized to [-pi, pi]."""
        return self.angle_normalize(a - b)

    def fix_map_to_odom_transform(self, *args):
        """Stub for fix_map_to_odom_transform. Does nothing."""
        pass


class FakeField:
    """Fake occupancy field: a 10 m x 10 m map with one wall along x = 5."""

    def get_closest_obstacle_distance(self, x, y):
        return abs(x - 5.0)

    def get_obstacle_bounding_box(self):
        return ((0.0, 10.0), (0.0, 10.0))


def make_pf(particles=None, n_particles=20):
    """Bare ParticleFilter with only what the tests need."""
    pf = ParticleFilter.__new__(ParticleFilter)
    pf.transform_helper = FakeTF()
    pf.occupancy_field = FakeField()
    pf.is_free = lambda x, y: True
    pf.odom_pose = (0.0, 0.0, 0.0)
    pf.particle_cloud = particles if particles is not None else []
    pf.n_particles = n_particles
    pf.sensor_noise = 0.2
    pf.beam_step = 5
    pf.random_reserve = 2
    pf.noise = 0.2
    pf.noise_theta = math.pi / 16
    return pf


def test_initialization_creates_n_particles():
    """Test that the particle filter initializes the correct number of particles."""
    pf = make_pf(n_particles=50)
    pf.initialize_particle_cloud(timestamp=None)
    assert len(pf.particle_cloud) == 50


def test_resampling_keeps_particle_count():
    """Test that resampling does not change the number of particles in the cloud."""
    cloud = [Particle(i * 0.5, 1.0, 0.0, w=float(i + 1)) for i in range(20)]
    pf = make_pf(cloud, n_particles=20)
    pf.resample_particles()
    assert len(pf.particle_cloud) == 20


def test_weights_sum_to_one():
    """Test that the weights of the particles sum to 1 after normalization."""
    pf = make_pf([Particle(w=w) for w in (1.0, 2.0, 5.0)], n_particles=3)
    pf.normalize_particles()
    assert sum(p.w for p in pf.particle_cloud) == pytest.approx(1.0)


def beams_used(capsys):
    """Pull N out of the 'beams: N' line printed by update_particles_with_laser."""
    for line in capsys.readouterr().out.splitlines():
        if line.startswith("beams:"):
            return int(line.split(":")[1])
    return None


def test_full_scan_uses_72_beams(capsys):
    """Test that a full 360-degree scan uses 72 beams with a beam_step of 5 degrees."""
    pf = make_pf([Particle(0.0, 0.0, 0.0)], n_particles=1)
    r = [5.0] * 360
    theta = [math.radians(i) for i in range(360)]
    pf.update_particles_with_laser(r, theta)
    assert beams_used(capsys) == 72
