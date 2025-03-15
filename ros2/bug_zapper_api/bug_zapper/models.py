# bug_zapper/models.py
from django.db import models

class MovingBug(models.Model):
    timestamp = models.DateTimeField(auto_now_add=True)
    radius = models.FloatField()
    speed = models.FloatField()
    position = models.JSONField(default=dict)  # Setze einen leeren Standardwert (leeres Dictionary)

    def __str__(self):
        return f"MovingBug {self.id} at {self.timestamp}"