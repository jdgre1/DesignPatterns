# bug_zapper/serializers.py
from rest_framework import serializers
from .models import MovingBug

class MovingBugSerializer(serializers.ModelSerializer):
    class Meta:
        model = MovingBug
        fields = ['id', 'timestamp', 'radius', 'speed', 'position']
