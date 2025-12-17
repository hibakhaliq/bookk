import React from 'react';
import Hero from '../components/Hero';
import chapters from '../data/chapters.js'; 

export default function Home() {
  return (
    <div>
      <Hero />

      {/* Chapters Section */}
      <section style={{ padding: '4rem 2rem', maxWidth: '1200px', margin: '0 auto' }}>
        <h2 style={{ textAlign: 'center', marginBottom: '3rem', fontSize: '2.5rem' }}>
          Chapters & Modules
        </h2>
        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
          gap: '2rem'
        }}>
          {chapters.map((chapter, idx) => (
            <div key={idx} style={{
              padding: '1.5rem',
              backgroundColor: '#f3f4f6',
              borderRadius: '1rem',
              boxShadow: '0 4px 12px rgba(0,0,0,0.05)'
            }}>
              <h3 style={{ marginBottom: '1rem', color: '#6366f1' }}>{chapter.title}</h3>
              <p style={{ opacity: 0.85 }}>{chapter.description}</p>
              <a href={chapter.link} style={{
                marginTop: '1rem',
                display: 'inline-block',
                color: 'white',
                backgroundColor: '#111827',
                padding: '0.5rem 1.2rem',
                borderRadius: '0.5rem',
                textDecoration: 'none'
              }}>Read Chapter</a>
            </div>
          ))}
        </div>
      </section>

      {/* Optional extra sections like ROS 2, Gazebo */}
      <section style={{ padding: '4rem 2rem', textAlign: 'center', backgroundColor: '#e0f2fe' }}>
        <h2 style={{ fontSize: '2rem', marginBottom: '1.5rem' }}>Extra Tutorials</h2>
        <p style={{ fontSize: '1.2rem' }}>ROS 2, Gazebo, Hands-on exercises, and more.</p>
      </section>
    </div>
  );
}
